import argparse
import rospy

def change_flight_state(args):
    rospy.set_param("/droneload/parameters/drone_state", args.state)

def set_window_id(args):
    rospy.set_param("/droneload/parameters/window_id", args.window_id)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ground stattion")
    
    subparsers = parser.add_subparsers(title="subcommands", dest="subcommand")
    
    switch_parser = subparsers.add_parser("switch", help="change flight state")
    switch_parser.add_argument("state", choices=[0, 1], type=int, help="flight state")
    switch_parser.set_defaults(func=change_flight_state)
    
    window_parser = subparsers.add_parser("window", help="set window id")
    window_parser.add_argument("window_id", type=int, help="window id")
    window_parser.set_defaults(func=set_window_id)
    
    args = parser.parse_args()

    if hasattr(args, "func"):
        args.func(args)
    else:
        parser.print_help()
    
    