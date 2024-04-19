# Job message definition
# Contains information about a new job obtained from the UI

# Job ID
uint32 job_id

# Job priority
uint8 priority

# Part
Part part

# Sub processes involved in the job
SubProcess[] subprocesses

uint64 start_time

uint64 end_time

# The status of the job - Can be "IN PROGRESS", "FINISHED", "PENDING", "FAILED", "REMOVED"
string status

# Whether a job is to be added or reomoved - Can be "ADD" or "REMOVE"
#string add_remove