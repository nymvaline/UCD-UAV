from Queue import *  # Uses built-in priority queue
import json  # To parse json setting file.

class TaskQueue(object):
    """
    TaskQueue
    Queue for managing tasks. Provides basic queue functions.
    """

    # Task Priorities
    HIGHEST = 0x0
    VERY_HIGH = 0x1
    HIGHER = 0x2
    HIGH = 0x3
    NORMAL = 0x4
    LOW = 0x5
    LOWER = 0x6
    VERY_LOW = 0x7
    LOWEST = 0x8

    def __init__(self):
        """
        Default Constructor. Creates an underlying PriorityQueue.
        :return: an instance of the TaskQueue.
        """
        self.q = PriorityQueue()

    def enqueue(self, task, priority = NORMAL):
        """
        Enqueue a task with priority.
        :param task: task to be enqueued.
        :param priority: priority of the enqueueing task. Default is TaskQueue.NORMAL.
        :except Queue.Full: if there are insufficient capacity to add a task into the queue.
        """
        try:
            self.q.put((priority, task))
        except Full:
            print("[INFO] TaskQueue is full. Ignoring task insertion.")

    def deque(self):
        """
        Returns and Deque the task with highest priority
        :return: task with highest priority defined in TaskQueue
        :except Queue.Empty: if there are no tasks in the queue.
        """
        try:
            return self.q.get()
        except Empty:
            print("[INFO] TaskQueue is empty. Ignoring task deque action.")



class Task(object):
    """
    Task
    Generic Task. To be sub-classed by application-specific tasks.
    """

    def __init__(self):
        """
        Default Constructor.
        :return: an instance of the Task.
        """
        pass

    def __execute__(self):
        """
        Executes the current task. To be overridden by children classes.
        """
        pass

    def __done__(self):
        """
        Interrogate the current task to see whether it has finished.
        :return: True or False.
        """
        pass