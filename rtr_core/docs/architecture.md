# Architecture: IEEE 1872.1 roles in `rtr_core`

`rtr_core` implements the task/goal/metric/constraint/outcome vocabulary of
IEEE Std 1872.1-2024 ("IEEE Standard for Robot Task Representation"). This
document shows where each of the standard's roles lives in the actual ROS2
node/process layout, and how a `TaskAttempt` action exchange flows between
them.

## Role → node mapping

| Standard role | Where it lives | `rtr_core` code |
|---|---|---|
| Specification | Task Client | builds the goal's `TaskSpecification` (metrics, constraints, `goal_params`, `completion_policy`) |
| User | Task Client | sends the goal, monitors `Feedback` |
| Approval | `AtomicTask` | `_goal_callback()` — ACCEPT / REJECT |
| Evaluation | `AtomicTask` | `TaskAssessor.assess()` via `self._assessor` |
| Task execution | `TaskExecutor` subclass | `validate_goal`, `initialize_goal`, `step`, `get_result` |

Three of the standard's five roles collapse into two ROS nodes: `AtomicTask`
plays both Approval and Evaluation, and a concrete `TaskExecutor` subclass
plays Task execution. The client plays both Specification and User.

```mermaid
flowchart TB
    subgraph ClientProc["Client process"]
        ClientNode["Task Client<br/>(ActionClient for TaskAttempt)"]
        SpecRole["Specification Role<br/>build the goal's TaskSpecification:<br/>metrics, constraints, goal_params,<br/>completion_policy"]
        UserRole["User Role<br/>send the goal,<br/>monitor Feedback"]
        ClientNode --- SpecRole
        ClientNode --- UserRole
    end

    subgraph ServerProc["Server process"]
        subgraph AtomicTaskNode["AtomicTask (rclpy Node)"]
            ApprovalRole["Approval Role<br/>_goal_callback():<br/>ACCEPT / REJECT"]
            EvalRole["Evaluation Role<br/>TaskAssessor.assess():<br/>Metric + Constraint eval at<br/>INITIALIZATION / EXECUTION / COMPLETION"]
        end
        subgraph ExecutorNode["TaskExecutor subclass (rclpy Node)"]
            ExecRole["Task Execution Role<br/>validate_goal, initialize_goal,<br/>step, get_result"]
        end
    end

    Environment["Environment / Hardware<br/>(task-specific sensors and actuators)"]

    UserRole ==>|"ROS2 action: <task_name>_attempt<br/>(TaskAttempt.Goal)"| ApprovalRole
    ApprovalRole -.->|"direct Python call<br/>(same process, not ROS)"| ExecRole
    EvalRole -.->|"direct Python call<br/>(same process, not ROS)"| ExecRole
    ExecRole ==>|"task-specific I/O<br/>(topics/services, implementation-defined)"| Environment
    Environment ==>|"task-specific I/O"| ExecRole
```

The dotted arrows are the important detail: `AtomicTask` and the
`TaskExecutor` subclass are two separate `rclpy` nodes, typically spun in the
same `MultiThreadedExecutor` in one process, but they never talk to each
other over ROS. `AtomicTask` holds a plain Python reference to the executor
(`self._executor`) and calls its methods directly. The only ROS2 IPC in this
picture is the action between the client and `AtomicTask`, and whatever
task-specific topics/services the executor uses to talk to its environment
(a real robot, a simulator, or something else entirely) — that boundary is
deliberately outside `rtr_core`'s concern.

## Message flow over one attempt

```mermaid
sequenceDiagram
    participant U as Task Client<br/>(Specification + User)
    participant AT as AtomicTask<br/>(Approval + Evaluation)
    participant EX as TaskExecutor<br/>(Task Execution)
    participant Env as Environment / Hardware

    Note over U,AT: ROS2 action "task_name_attempt"
    U->>AT: send_goal(TaskAttempt.Goal{specification})
    AT->>EX: GoalParamModel.model_validate_json(goal_params)
    AT->>EX: validate_goal(goal_params)
    EX-->>AT: True / False

    alt rejected
        AT-->>U: GoalResponse.REJECT
    else accepted
        AT-->>U: GoalResponse.ACCEPT
        Note over AT: goal_handle.execute() (rclpy default)<br/>triggers _execute_callback
        AT->>EX: initialize_goal(goal_params)
        EX-->>AT: initial_state
        AT->>AT: assess(initial_state, STAGE_INITIALIZATION)

        loop each control cycle
            AT->>EX: step()
            EX->>Env: task-specific I/O
            Env-->>EX: task-specific I/O
            EX-->>AT: state
            AT->>AT: assess(state, STAGE_EXECUTION)
            AT-->>U: publish_feedback(Assessment, feedback_params)
        end

        AT->>EX: get_result(canceled)
        EX-->>AT: (final_state, result_params)
        AT->>AT: assess(final_state, STAGE_COMPLETION)
        AT-->>U: result(exit, effect, assessment)
    end
```

Cancellation is a side channel not shown above: the client calls
`cancel_goal()`, `AtomicTask` calls `on_cancel()` on the executor, and the
execute loop notices `goal_handle.is_cancel_requested` on its next iteration
and breaks out into the same `get_result(canceled=True)` path.
