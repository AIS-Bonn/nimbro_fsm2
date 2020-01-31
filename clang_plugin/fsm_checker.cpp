// Clang plugin for nimbro_fsm2
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <iterator>
#include <memory>
#include <set>
#include <regex>

#include <llvm/Support/CommandLine.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcomment"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#include <clang/Frontend/FrontendPluginRegistry.h>
#include <clang/Frontend/CompilerInstance.h>
#include <clang/AST/ASTConsumer.h>
#include <clang/AST/RecursiveASTVisitor.h>
#include <clang/ASTMatchers/ASTMatchers.h>
#include <clang/ASTMatchers/ASTMatchFinder.h>
#include <clang/Basic/Version.h>

#pragma GCC diagnostic pop

namespace
{

using namespace clang;
using namespace clang::ast_matchers;

template<class T>
auto getLoc(T& node)
{
#if CLANG_VERSION_MAJOR < 7
	return node.getLocStart();
#else
	return node.getBeginLoc();
#endif
}

using TypeList = std::vector<const Type*>;

auto stateConstructor = allOf(
	isDefinition(),
	unless(isDefaulted()),
	ofClass(isDerivedFrom("nimbro_fsm2::detail::AbstractState"))
);

auto InterestingConstructorMatcher = cxxConstructorDecl(
	stateConstructor,
	hasBody(
		hasDescendant(
			callExpr().bind("call")
		)
	)
);

auto ProblematicConstructorMatcher = cxxConstructorDecl(
	stateConstructor,
	hasBody(
		hasDescendant(
			callExpr(
				callee(
					cxxMethodDecl(hasName("driver"))
				)
			).bind("call")
		)
	)
);

auto FullySpecifiedState = cxxRecordDecl(
	isDerivedFrom("nimbro_fsm2::detail::AbstractState"),
	isDefinition()
).bind("state");

auto InitializeCallMatcher = cxxMemberCallExpr(callee(cxxMethodDecl(

	// match nimbro_fsm2::FSM::initialize()
	hasName("initialize"), ofClass(cxxRecordDecl(hasName("::nimbro_fsm2::FSM"))),

	// retrieve type of local typedef "ReachableStates"
	hasDescendant(typeAliasDecl(
		// extract the decl nimbro_fsm2::detail::Collector<State...>
		hasType(elaboratedType(namesType(type(hasUnqualifiedDesugaredType(
			recordType(hasDeclaration(
				classTemplateSpecializationDecl().bind("reachableStates")
			))
		))))),
		hasName("ReachableStates")
	))
)));

auto ProblematicEnterMatcher = cxxMethodDecl(
	ofClass(cxxRecordDecl(
		isDerivedFrom("nimbro_fsm2::detail::AbstractState")
	).bind("class")),

	hasName("enter"),

	// We override a non-trivial enter() (or do not have its definition)
	forEachOverridden(cxxMethodDecl(
		anyOf(
			hasBody(stmt(hasDescendant(stmt()))),
			unless(hasBody(anything()))
		)
	)),

	// ... and we don't call it.
	hasBody(
		stmt(
			unless(
				hasDescendant(
					callExpr(
						callee(
							cxxMethodDecl(hasName("enter"))
						)
					)
				)
			)
		).bind("definition")
	)
);

auto ProblematicLeaveMatcher = cxxMethodDecl(
	ofClass(cxxRecordDecl(
		isDerivedFrom("nimbro_fsm2::detail::AbstractState")
	).bind("class")),

	hasName("leave"),

	// We override a non-trivial leave() (or do not have its definition)
	forEachOverridden(cxxMethodDecl(
		anyOf(
			hasBody(stmt(hasDescendant(stmt()))),
			unless(hasBody(anything()))
		)
	)),

	// ... and we don't call it.
	hasBody(
		stmt(
			unless(
				hasDescendant(
					callExpr(
						callee(
							cxxMethodDecl(hasName("leave"))
						)
					)
				)
			)
		).bind("definition")
	)
);

const char* PLUGIN_NAME = "nimbro_fsm2";

class InterestingHandler : public MatchFinder::MatchCallback
{
public:
	explicit InterestingHandler(CompilerInstance& C)
	 : m_C{C}
	{
		m_warning = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Warning,
			"You are calling functions inside your state constructor. This is "
			"not recommended, since the nimbro_fsm2 clang plugin cannot check "
			"if you mistakenly call driver() inside these functions. "
			"Consider doing this work inside the State::enter() method."
		);
	}

	void run (const MatchFinder::MatchResult& result) override
	{
		auto call = result.Nodes.getNodeAs<CallExpr>("call");
		if(!call)
			std::abort();

		auto diag = m_C.getDiagnostics().Report(getLoc(*call), m_warning);
	}
private:
	CompilerInstance& m_C;
	unsigned int m_warning;
};

class ProblematicHandler : public MatchFinder::MatchCallback
{
public:
	explicit ProblematicHandler(CompilerInstance& C)
	 : m_C{C}
	{
		m_error = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Error,
			"You are calling driver() in the constructor of your state class. "
			"This is not allowed. Override State::enter() and perform your "
			"work there."
		);
	}

	void run (const MatchFinder::MatchResult& result) override
	{
		auto call = result.Nodes.getNodeAs<CallExpr>("call");
		if(!call)
			std::abort();

		auto diag = m_C.getDiagnostics().Report(getLoc(*call), m_error);
	}
private:
	CompilerInstance& m_C;
	unsigned int m_error;
};

class StateHandler : public MatchFinder::MatchCallback
{
public:
	void run(const MatchFinder::MatchResult& result) override
	{
		auto state = result.Nodes.getNodeAs<CXXRecordDecl>("state");
		if(!state)
			std::abort();

		if(state->isAbstract())
			return;

		m_states.push_back(state->getTypeForDecl());
	}

	const TypeList& states() const
	{ return m_states; }
private:
	TypeList m_states;
};

class InitializeHandler : public MatchFinder::MatchCallback
{
public:
	void run(const MatchFinder::MatchResult& result) override
	{
		auto collector = result.Nodes.getNodeAs<ClassTemplateSpecializationDecl>("reachableStates");
		if(!collector)
			std::abort();

		m_matched = true;

		auto& args = collector->getTemplateInstantiationArgs();

		if(args.size() == 0)
			return;

		auto& pack = args[0];

		if(pack.getKind() != TemplateArgument::Pack)
			std::abort();

		for(auto& arg : pack.getPackAsArray())
		{
			m_reachableStates.push_back(arg.getAsType().getTypePtr());
		}
	}

	bool matched() const
	{ return m_matched; }

	const TypeList& reachableStates() const
	{ return m_reachableStates; }
private:
	bool m_matched = false;
	TypeList m_reachableStates;
};

class TransitCollector : public MatchFinder::MatchCallback
{
public:
	void run(const MatchFinder::MatchResult& result) override
	{
		auto type = result.Nodes.getNodeAs<RecordType>("state");
		if(!type)
			std::abort();

		m_transitions.insert(type);
	}

	const std::set<const RecordType*>& transitions() const
	{ return m_transitions; }
private:
	std::set<const RecordType*> m_transitions;
};

class ProblematicEnterHandler : public MatchFinder::MatchCallback
{
public:
	explicit ProblematicEnterHandler(CompilerInstance& C)
	 : m_C{C}
	{
		m_error = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Warning,
			"Your base state (or one of your ancestor states) has a non-trivial "
			"enter() method. You are not calling it in your enter() method. "
			"This is probably a bug."
		);
	}

	void run(const MatchFinder::MatchResult& result) override
	{
		auto call = result.Nodes.getNodeAs<Stmt>("definition");
		if(!call)
			std::abort();

		auto myClass = result.Nodes.getNodeAs<CXXRecordDecl>("class");
		if(!myClass)
			std::abort();

		auto diag = m_C.getDiagnostics().Report(getLoc(*call), m_error);

		if(myClass->getNumBases() == 1)
		{
			diag << clang::FixItHint::CreateInsertion(getLoc(*call),
				myClass->bases_begin()->getType().getAsString() + "::enter();"
			);
		}
	}
private:
	CompilerInstance& m_C;
	unsigned int m_error;
};

class ProblematicLeaveHandler : public MatchFinder::MatchCallback
{
public:
	explicit ProblematicLeaveHandler(CompilerInstance& C)
	 : m_C{C}
	{
		m_error = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Warning,
			"Your base state (or one of your ancestor states) has a non-trivial "
			"leave() method. You are not calling it in your leave() method. "
			"This is probably a bug."
		);
	}

	void run(const MatchFinder::MatchResult& result) override
	{
		auto call = result.Nodes.getNodeAs<Stmt>("definition");
		if(!call)
			std::abort();

		auto myClass = result.Nodes.getNodeAs<CXXRecordDecl>("class");
		if(!myClass)
			std::abort();

		auto diag = m_C.getDiagnostics().Report(getLoc(*call), m_error);

		if(myClass->getNumBases() == 1)
		{
			diag << clang::FixItHint::CreateInsertion(getLoc(*call),
				myClass->bases_begin()->getType().getAsString() + "::leave();"
			);
		}
	}
private:
	CompilerInstance& m_C;
	unsigned int m_error;
};

class Consumer : public clang::ASTConsumer
{
public:
	explicit Consumer(CompilerInstance& C)
	 : m_C{C}
	{
		m_warning = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Warning,
			"This state is not reachable. Suggestion: Either create a "
			"transition to it or add it to the list of start states in the "
			"FSM::initialize() call."
		);

		m_transitWarning = C.getDiagnostics().getCustomDiagID(
			DiagnosticsEngine::Warning,
			"Transition %0 is specified in the template arguments, but it seems "
			"it is never used. Consider removing it."
		);
	}

	void checkStateDefRecurse(MatchFinder& finder, const CXXRecordDecl& classDecl, ASTContext& context)
	{
// 		llvm::errs() << "Recursing into " << classDecl.getName() << ", isComplete: " << classDecl.isCompleteDefinition() << "\n";

		for(auto* method : classDecl.methods())
		{
			auto definition = method->getDefinition();
			if(!definition)
			{
// 				llvm::errs() << "method " << method->getName() << " has no definition\n";
// 				llvm::errs() << "late: " << method->isLateTemplateParsed() << "\n";
				continue;
			}

			finder.match(*definition, context);
		}

		for(auto base : classDecl.bases())
			checkStateDefRecurse(finder, *base.getType().getTypePtr()->getAsCXXRecordDecl()->getDefinition(), context);
	}

	void checkStateDefinition(clang::ASTContext& context, const CXXRecordDecl& classDecl)
	{
		// Heuristic: we check state definitions that are a) completely defined
		// or b) have definitions in our main file.
		// NOTE: a) is disabled for now, since it gives false positives on
		//  states defined purely in the header, which are not fully
		//  instantiated.

		if(classDecl.isAbstract())
			return;

		bool inMainFile = false;
		bool completelyDefined = true;

		auto& sourceManager = context.getSourceManager();

		for(auto* method : classDecl.methods())
		{
			auto definition = method->getDefinition();
			if(!definition)
			{
				completelyDefined = false;
				continue;
			}

			if(sourceManager.isInMainFile(sourceManager.getExpansionLoc(getLoc(*definition))))
				inMainFile = true;
		}

		if(/*!completelyDefined &&*/ !inMainFile)
			return;

		// We now collect all transit() calls
		TransitCollector collector;
		{
			MatchFinder finder;

			auto transitMatcher = cxxMethodDecl(forEachDescendant(cxxMemberCallExpr(callee(
				cxxMethodDecl(
					hasName("transit"),
					ofClass(cxxRecordDecl(hasName("State"))),
					hasTemplateArgument(0, refersToType(
						type().bind("state")
					))
				)
			))));

			finder.addMatcher(transitMatcher, &collector);

			// I haven't found a way to traverse getDefinition() using AST matchers,
			// so do that manually here.

			checkStateDefRecurse(finder, classDecl, context);
		}

		// Find user-specified info
		auto specMatcher = cxxRecordDecl(isDerivedFrom(cxxRecordDecl(
			hasDescendant(typeAliasDecl(
				hasName("Transitions"),
				hasType(hasUnqualifiedDesugaredType(recordType(hasDeclaration(
					classTemplateSpecializationDecl().bind("transitions")
				))))
			))
		)));

		auto specList = match(specMatcher, classDecl, context);
		if(specList.empty())
			std::abort();

		auto& spec = specList.front();

		auto transitArgs = spec.getNodeAs<ClassTemplateSpecializationDecl>("transitions");
		if(!transitArgs || transitArgs->getTemplateInstantiationArgs().size() == 0)
			std::abort();

		auto specTransits = transitArgs->getTemplateInstantiationArgs()[0];

		std::set<const RecordType*> specTypes;
		for(auto& arg : specTransits.getPackAsArray())
		{
			auto type = arg.getAsType().getTypePtr();
			if(type->isRecordType())
				specTypes.insert(static_cast<const RecordType*>(type));
		}

		auto& detectedTransits = collector.transitions();
		for(auto& specType : specTypes)
		{
			auto it = std::find(detectedTransits.begin(), detectedTransits.end(), specType);
			if(it == detectedTransits.end())
			{
				auto diag = m_C.getDiagnostics().Report(
					getLoc(classDecl),
					m_transitWarning
				);
				diag.AddString(specType->getDecl()->getName());
			}
		}
	}

	void HandleTranslationUnit(clang::ASTContext &Context) override
	{
		MatchFinder finder;
		InterestingHandler handler{m_C};
		ProblematicHandler phandler{m_C};
		InitializeHandler ihandler;
		StateHandler stateHandler;
		ProblematicEnterHandler enterHandler{m_C};
		ProblematicLeaveHandler leaveHandler{m_C};

		finder.addMatcher(ProblematicConstructorMatcher, &phandler);
		finder.addMatcher(InterestingConstructorMatcher, &handler);
		finder.addMatcher(InitializeCallMatcher, &ihandler);
		finder.addMatcher(FullySpecifiedState, &stateHandler);
		finder.addMatcher(ProblematicEnterMatcher, &enterHandler);
		finder.addMatcher(ProblematicLeaveMatcher, &leaveHandler);
		finder.matchAST(Context);

		if(ihandler.matched())
		{
			auto& reachable = ihandler.reachableStates();

			for(const Type* state : stateHandler.states())
			{
				auto it = std::find(reachable.begin(), reachable.end(), state);
				if(it == reachable.end())
				{
					auto diag = m_C.getDiagnostics().Report(getLoc(*state->getAsCXXRecordDecl()), m_warning);
				}
			}
		}

		for(const Type* state : stateHandler.states())
			checkStateDefinition(Context, *state->getAsCXXRecordDecl());
	}
private:
	CompilerInstance& m_C;
	unsigned int m_warning;
	unsigned int m_transitWarning;
};

class Plugin : public clang::PluginASTAction
{
public:
	Plugin()
	 : clang::PluginASTAction()
	{}

	Plugin(Plugin const &) = delete;
	Plugin & operator=(Plugin const &) = delete;

private:
	// automatically run this plugin after creating the AST
	ActionType getActionType() override
	{
		return AddAfterMainAction;
	}

	std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(clang::CompilerInstance& C, llvm::StringRef) override
	{
		return std::make_unique<Consumer>(C);
	}

	bool ParseArgs(const clang::CompilerInstance&,
				   const std::vector<std::string>& Args) override
	{
		std::regex versionRegex(R"EOS(clang version (\d+).(\d+).(\d+).*)EOS");
		std::string version = clang::getClangFullVersion();
		std::smatch match;

		if(std::regex_match(version, match, versionRegex))
		{
			int major = std::atoi(match[1].str().c_str());
			int minor = std::atoi(match[2].str().c_str());
			if(major != CLANG_VERSION_MAJOR || minor != CLANG_VERSION_MINOR)
			{
				fprintf(stderr, "Version mismatch: the nimbro_fsm2 checker was compiled with Clang %d.%d, but we are running under %d.%d!\n",
					CLANG_VERSION_MAJOR, CLANG_VERSION_MINOR,
					major, minor
				);
				return false;
			}
		}
		else
			fprintf(stderr, "Warning: Could not parse clang version '%s'\n", version.c_str());

		return true;
	}

private:
};

} // namespace anonymous

static clang::FrontendPluginRegistry::Add<Plugin>
Register(PLUGIN_NAME, "Check nimbro_fsm2 usage");
