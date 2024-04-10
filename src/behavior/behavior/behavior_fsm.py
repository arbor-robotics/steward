from transitions import Machine, State
from transitions.extensions import GraphMachine
import random


class StewardFSM(object):

    # Define some states. Most of the time, narcoleptic superheroes are just like
    # everyone else. Except for...
    states = [
        State("PAUSED"),
        State("DRIVING", on_enter="callDrivingAction"),
        State("PLANTING", on_enter="callPlantingAction"),
    ]

    def __init__(self, onDrivingStart):

        self.onDrivingStart = onDrivingStart

        # What have we accomplished today?
        self.trees_planted = 0

        # Initialize the state machine
        self.machine = GraphMachine(
            model=self, states=StewardFSM.states, initial="PAUSED"
        )

        # Add some transitions. We could also define these using a static list of
        # dictionaries, as we did with states above, and then pass the list to
        # the Machine initializer as the transitions= argument.

        # At some point, every superhero must rise and shine.
        self.machine.add_transition(
            trigger="start", source="PAUSED", dest="DRIVING", conditions=["is_healthy"]
        )

        self.machine.add_transition(
            "goalReached", "DRIVING", "PLANTING", conditions=["is_healthy"]
        )
        self.machine.add_transition(
            "treePlanted", "PLANTING", "DRIVING", conditions=["is_healthy"]
        )

        self.machine.add_transition("onError", "*", "PAUSED")
        self.machine.add_transition("pause", "*", "PAUSED")

    def callPlantingAction(self):
        print("Calling planting action")

    def callDrivingAction(self):
        self.onDrivingStart()
        print("Calling driving action")

    @property
    def is_healthy(self):
        """Basically a coin toss."""
        return random.random() < 0.5

    def change_into_super_secret_costume(self):
        print("Beauty, eh?")


if __name__ == "__main__":
    machine = StewardFSM()
    machine.get_graph().draw("my_state_diagram.png", prog="dot")
    machine.start()

    for i in range(10):
        print(machine.state)
        if machine.state == "PAUSED":
            machine.start()
        elif machine.state == "DRIVING":
            machine.goalReached()
        elif machine.state == "PLANTING":
            machine.treePlanted()
