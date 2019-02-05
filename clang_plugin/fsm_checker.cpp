// Clang plugin for nimbro_fsm2
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <iterator>
#include <memory>

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
	}

	void HandleTranslationUnit(clang::ASTContext &Context) override
	{
		MatchFinder finder;
		InterestingHandler handler{m_C};
		ProblematicHandler phandler{m_C};
		InitializeHandler ihandler;
		StateHandler stateHandler;

		finder.addMatcher(ProblematicConstructorMatcher, &phandler);
		finder.addMatcher(InterestingConstructorMatcher, &handler);
		finder.addMatcher(InitializeCallMatcher, &ihandler);
		finder.addMatcher(FullySpecifiedState, &stateHandler);
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
	}
private:
	CompilerInstance& m_C;
	unsigned int m_warning;
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
		return true;
	}

private:
};

} // namespace anonymous

static clang::FrontendPluginRegistry::Add<Plugin>
Register(PLUGIN_NAME, "Check nimbro_fsm2 usage");
