int8 STAGE_INITIALIZATION=1
int8 STAGE_EXECUTION=2
int8 STAGE_COMPLETION=3

# A unique identifying name for the Constraint.
string name

# The comparison that establishes this constraint.
SerializedData constraint_params

# The task accomplishment stages during which this constraint applies
int8[] active_stages