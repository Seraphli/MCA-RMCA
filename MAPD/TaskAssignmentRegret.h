//
// Created by Zhe Chen on 10/6/20.
//

#ifndef LIFELONG_MAPF_TaskAssignmentRegret_H
#define LIFELONG_MAPF_TaskAssignmentRegret_H

#include "Agent.h"
#include "Task.h"
#include "PathPlanning.h"
#include "Assignment.h"
#include "ConstraintTable.h"
#include <unordered_set>
#include <unordered_map>
#include "common.h"
#include "SinglePlanning.h"
#include "TaskAssignment.h"


class TaskAssignmentRegret: public TaskAssignment{
public:


    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap_regret>> Task_heap_regret;
    typedef boost::heap::fibonacci_heap<AssignmentHeap*,boost::heap::compare<compare_assignment_heap_regret>>::handle_type  TaskHeap_handle_regret;

    TaskAssignmentRegret(AgentLoader* agents, TaskLoader* tasks, MapLoaderCost* map, TA_Options ta_opt, options pl_option, bool real_cost, float pl_time_limit)
        :TaskAssignment(agents, tasks, map, ta_opt, pl_option, real_cost, pl_time_limit){};

    void initialize_heaps(){
        handleTable.clear();
        for(auto& t: unassigned_tasks){
            handleTable[t->task_id].resize(agents->num_of_agents);
        }

        allAssignmentHandles.clear();
    };
    Assignment* get_best_assignment() override{
        // Safety check to ensure the heap is not empty
        if (allAssignmentHeap.empty()) {
            return nullptr; // Handle this case appropriately in the caller
        }
        AssignmentHeap* top_heap = allAssignmentHeap.top();
        if (top_heap == nullptr || top_heap->empty()) {
            return nullptr; // Handle this case appropriately in the caller
        }
        return top_heap->top();
    };
    
    void delete_top(int task_id) override{
        // Safety check to ensure the heap is not empty
        if (allAssignmentHeap.empty()) {
            handleTable[task_id].clear();
            return;
        }
        
        AssignmentHeap* top_heap = allAssignmentHeap.top();
        if (top_heap != nullptr) {
            for (auto assign : *top_heap){
                delete assign;
            }
            delete top_heap;
        }
        allAssignmentHeap.pop();
        handleTable[task_id].clear();
    }
    void logAssignmentHandle(Task* task,Agent* a,AssignmentHeap* new_assignment_heap){
        allAssignmentHandles[task->task_id] = allAssignmentHeap.push(new_assignment_heap);
    };
    bool updateRealPath(Assignment* min_cost_assign,Agent* a,int next_action = 1){
            min_cost_assign->current_total_delay = prioritized_planning(min_cost_assign->path, min_cost_assign->actions, min_cost_assign->agent,next_action);
            min_cost_assign->cost_increase = min_cost_assign->current_total_delay - assignments[a->agent_id].current_total_delay;
            if(min_cost_assign->cost_increase<0)
                min_cost_assign->cost_increase = 1.0/(float)abs(min_cost_assign->cost_increase);
            min_cost_assign->makespan = min_cost_assign->actions[min_cost_assign->actions.size() - 2].real_action_time;

            if (ta_option.objective == OBJECTIVE::MAKESPAN) {
                min_cost_assign->cost_increase =
                        ( min_cost_assign->makespan -
                          current_makespan) > 0 ? (min_cost_assign->makespan - current_makespan) : 0;
            }
            return true;
    }

    void updateAllAssignmentHeap(Agent* updatedAgent,Task* assignedTask);
    void printTaskHeap() override {
        if (allAssignmentHeap.empty()) {
            cout << "Task heap is empty" << endl;
            return;
        }
        
        for(auto it = allAssignmentHeap.ordered_begin(); it != allAssignmentHeap.ordered_end(); ++it){
            auto agentHeap = (*it);
            
            if (agentHeap == nullptr || agentHeap->empty()) {
                cout << " Task : <empty heap>" << endl;
                continue;
            }
            
            Assignment* top_assign = agentHeap->top();
            if (top_assign == nullptr) {
                cout << " Task : <null assignment>" << endl;
                continue;
            }
            
            cout << " Task : " << top_assign->new_add_task->task_id;
            cout << " MC : " << top_assign->cost_increase;

            for (auto agit = agentHeap->ordered_begin(); agit != agentHeap->ordered_end(); ++agit){
                auto assign = (*agit);
                if (assign != nullptr) {
                    cout << ", Agent: " << assign->agent->agent_id << " MC: " << assign->cost_increase;
                }
            }
            cout << endl;
        }
    }


    Task_heap_regret allAssignmentHeap;
    std::unordered_map<int,TaskHeap_handle_regret> allAssignmentHandles;





};

#endif //LIFELONG_MAPF_TaskAssignmentRegret_H
