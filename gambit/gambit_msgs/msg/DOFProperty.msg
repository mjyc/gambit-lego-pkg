# Represents a property of a DOF

uint8 DOF_index               # index of associated DOF
string property_name        # string identifier for property
string display_name         # human-readable display name for property
string type                 # type of property (bool, string, int, etc.)
string rw                   # either r, w, rw
string string_value
int32 int_value
float32 float_value

