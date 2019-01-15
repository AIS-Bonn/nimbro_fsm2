
# The current state name.
# This is also the name of the last element in history.
string current_state

# A free text field that can be used by states to display values to the user.
string display_messages

# Is the FSM paused?
bool paused

# History of the past N states
StateStatus[] history
