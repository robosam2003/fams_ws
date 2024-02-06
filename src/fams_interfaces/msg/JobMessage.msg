# Job message definition
# Contains information about a new job obtained from the UI

# Job ID
uint32 job_id

# Job priority
uint8 priority

# Sub processes involved in the job
SubProcess[] subprocesses

# The times of the subprocesses
uint64[] subprocess_start_times

uint64[] subprocess_end_times

uint64 job_start_time

uint64 job_end_time

# The status of the job - Can be "ACTIVE", "COMPLETED", "FAILED""
string status