# ID
uint32 workstation_id

string state # Can be "FREE", "BUSY", "BROKEN"

# List of operations that the workstation can perform (milling, drilling, etc.)
uint32[] available_operations 