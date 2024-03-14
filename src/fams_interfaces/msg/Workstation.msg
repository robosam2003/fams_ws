# ID
uint32 workstation_id

string state # Can be "FREE", "BUSY", "BROKEN"

# List of operations that the workstation can perform (milling, drilling, etc.)
string[] available_operations 

string status