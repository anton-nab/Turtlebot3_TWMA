import sys
import time

class TurtleRobotControl:
    def __init__(self):
        self.choose_mode = 0

    def get_mode(self):
        while True:
            if self.choose_mode == 1:
                self.launch_follow_me()
            elif self.choose_mode == 2:
                self.launch_navigation()
            else:
                print("attente de sélection d'un mode")
                time.sleep(5000)

    def launch_follow_me(self):
        print("Mode follow me en cours")
        time.sleep(5)
          

    def launch_navigation(self):
        print("Mode navigation par points en cours")
        time.sleep(5)
        
          

            


def main():
    robot = TurtleRobotControl()

    result = robot.play()

    sys.exit()

# if python says run, then we should run
if __name__ == "__main__":
    main()