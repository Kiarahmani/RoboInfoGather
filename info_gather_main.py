from pomdp_exec import *
from groundingdino.util.inference import load_model
from program_utils import *

def main(nl):
    """
    Main loop for info gathering. Given a natural language query, generate a program in our dsl.
    Then foreach query, generate a POMDP for the query and execute it.
    After executing all queries, evaluate their symbolic results and return the desired information

    :param: (nl) Natural language query representing the information to be gathered

    :return: (query_results)
    """

    # Load the config
    config_filename = os.path.join(f"info_gather.yaml")
    config = yaml.load(open(config_filename, "r"), Loader=yaml.FullLoader)

    # check if we want to quick load or full load the scene
    load_options = {
        "Quick": "Only load the building assets (i.e.: the floors, walls, doors)",
        "Full": "Load all interactive objects in the scene",
    }
    load_mode = choose_from_options(options=load_options, name="load mode", random_selection=random_selection)
    if load_mode == "Quick":
        config["scene"]["load_object_categories"] = ["floors", "walls", "door"]

    # Load the environment
    env = og.Environment(configs=config)

    # Allow user to move camera more easily
    og.sim.enable_viewer_camera_teleoperation()

    # Reset env before start? 
    assert False #Do we need to do this?
    og.log.info("Resetting environment")
    env.reset()

    # Generate program
    prog = gen_prog_from_nl(nl)

    # Set up models
    dino_model = load_model(CONFIG_PATH, WEIGHTS_PATH)

    # Execute each query
    query_results = []
    for query in prog:
        assert False # Will this be a loop??
        pomdp = gen_pomdp_from_query(query)

        symbolic_info = pomdp_exec_loop(env, pomdp, config)

        query_results.append(eval_query(query, symbolic_info))

    # Always close the environment at the end
    env.close()

    return query_reults