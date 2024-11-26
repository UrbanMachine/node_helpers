# A prompt that only provides options to a user on some arbitrary menu implementation
# Used by `node_helpers.interaction`


# Metadata schema:
#   No members
uint8 PROMPT_BASIC = 0
# A prompt used to teleop a hardware component. When the dashboard receives a
# prompt with this type, it will display menus to teleop the specified
# components.
#
# Metadata schema:
#   namespaces - An array of hardware module namespaces, specifying what
#                hardware should be teleoped
uint8 PROMPT_TELEOP = 1

# Available options to transition to other states
PromptOption[] options
# Provides additional context about the prompt
string help
# The prompt type. Effects what additional options are presented to the user by
# the dashboard
uint8 type 0
# A JSON object whose contents depend on the type of prompt being used
string metadata
