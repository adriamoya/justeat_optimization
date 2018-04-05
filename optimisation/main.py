
import os
import json
from ortools.constraint_solver import pywrapcp


def load_data(exercise):
    """Load JSON data for a given exercise"""

    json_files = ['ex1-in.json', 'ex2-in.json', 'ex3-in.json']
    path_to_json = 'planning_demand/'

    if exercise == "ex1":
        with open(os.path.join(path_to_json, json_files[0])) as json_file:
            json_data = json.load(json_file)
    elif exercise == "ex2":
        with open(os.path.join(path_to_json, json_files[1])) as json_file:
            json_data = json.load(json_file)
    elif exercise == "ex3":
        with open(os.path.join(path_to_json, json_files[2])) as json_file:
            json_data = json.load(json_file)

    return json_data


def optimize_shifts(exercise, drivers):

    ## Creating the solver and shift matrix
    ## ------------------------------------------------------

    # Create the solver.
    solver = pywrapcp.Solver(exercise)

    print "\n", "*"*100, "\nExercise %s \n" % exercise[-1], "*"*100

    # Set Variables.
    num_vals  = 2 # [0, 1] driver working or not
    json_data = load_data(exercise)
    demand    = json_data['demand'] # demand forecast
    slots     = len(demand) # number of timeslots
    min_h     = 4 # minimum number of hours per shift
    max_h     = 10 # maximum number of hours per shift

    print "\nNumber of drivers:", drivers

    # Create shift variables.
    shifts = {}
    for j in range(drivers):
        for i in range(slots):
            shifts[(j,i)]=solver.IntVar(0, num_vals-1, "d%i_slot%i" % (j,i))

    # Flattened version of shifts (required by Phase method).
    s_flat = [shifts[(j,i)] for j in range(drivers) for i in range(slots)]


    ## Constraints
    ## ------------------------------------------------------

    # Demand by timeslot.
    for i in range(slots):
        solver.Add(solver.Sum([shifts[(j,i)] for j in range(drivers)]) >= demand[i])

    # Maximum and minimum duration of shifts per driver.
    for j in range(drivers):
        solver.Add(solver.Sum([shifts[(j,i)] for i in range(slots)]) >= min_h)
        solver.Add(solver.Sum([shifts[(j,i)] for i in range(slots)]) <= max_h)

    # Consecutive hours working (no gaps, one shift per driver).
    for j in range(drivers):
        for i in range(slots-2):
            solver.Add(solver.Max(shifts[(j, i)] == shifts[(j, i+1)], shifts[(j, i+1)] == shifts[(j, i+2)]) == 1)
        for i in range(slots-4):
            solver.Add(solver.Max(shifts[(j, i)] == shifts[(j, i+2)], shifts[(j, i+2)] == shifts[(j, i+4)]) == 1)


    # Define objective function
    ## ------------------------------------------------------

    obj_var = solver.Sum([shifts[(j,i)]for j in range(drivers) for i in range(slots)])
    obj_monitor = solver.Minimize(obj_var, 1)
 

    ## Find soltions and display results
    ## ------------------------------------------------------

    # Create decision builder.
    db = solver.Phase(s_flat, 
                      solver.CHOOSE_FIRST_UNBOUND, 
                      solver.ASSIGN_MIN_VALUE)
    
    # Set time limit.
    time_limit = 15000
    time_limit_ms = solver.TimeLimit(time_limit)

    # Add variables to the solution collector.
    solution = solver.Assignment()
    solution.Add(s_flat)
    collector = solver.FirstSolutionCollector() # AllSolutionCollector
    collector.Add(s_flat)
    collector.AddObjective(obj_var)

    solver.Solve(db, [obj_monitor, time_limit_ms, collector])

    if solver.Solve(db, [time_limit_ms, collector]):
        print "Total demand hours:", sum(demand)
        print "Optimal amount of estimated hours:", collector.ObjectiveValue(0)
        print "Solutions found in %s seconds: " % str(time_limit/1000), collector.SolutionCount()
        print "Time:", solver.WallTime(), "ms"
        print "\n", "*"*100, "\n"



        for j in range(drivers):
            l = []
            for i in range(slots):
                l.append(collector.Value(0, shifts[(j, i)]))
            print l

        t = []
        for i in range(slots):
            l = 0
            for j in range(drivers):
                l +=collector.Value(0, shifts[(j, i)])
            t.append(l)
        print "\nEstimated drivers per timeslot:"
        print t
        print "\nForecasted demand:"
        print demand

        print "\nOversupply hours:", str(collector.ObjectiveValue(0)), "-", str(sum(demand)), "=", str(collector.ObjectiveValue(0)-sum(demand))

    else:
        print "\n", "*"*100, "\n"
        print "No solution found. Time limit set to %s seconds." % str(time_limit/1000)

    print ""
