from enum import Enum


# A work cell is a physical location in the factory where a robot can be assigned to perform a task
class WorkCell(Enum):
    UNDEFINED = 0
    INSPECTION = 6
    ROBOT_ARM_1 = 1
    ROBOT_ARM_2 = 2
    DEPOT = 4
    ROBOT_ARM_3 = 5


# An AMR is an autonomous mobile robot. We currently have the following 2 robots at the testbed
class AMR(Enum):
    AMR_1 = 1
    AMR_2 = 2


class TaskStatus(Enum):
    BACKLOG = 1
    ENQUEUED = 2
    RUNNING = 3
    COMPLETED = 4
    FAILED = 5
    CANCELED = 6


class TestbedTaskType(Enum):
    UNLOADING = 1
    PROCESSING = 2
    LOADING = 3


class AssemblyType(Enum):
    M = 1
    F = 2
    I = 3