import logging
import subprocess
from pathlib import Path
from typing import Any

from node_helpers_msgs.msg import PlaySound
from pydantic import BaseModel, DirectoryPath
from rclpy.qos import qos_profile_services_default

from node_helpers.nodes.helpful_node import HelpfulNode
from node_helpers.spinning import create_spin_function


class SoundPlayer(HelpfulNode):
    """
    A generic node for requesting sound effects to be played via topics.
    """

    class Parameters(BaseModel):
        sound_effects_directory: DirectoryPath = Path("config/common/sound_effects/")
        """A directory with *.ogg sound files"""

    def __init__(self, **kwargs: Any):
        super().__init__("sound_player", **kwargs)

        self.params = self.declare_from_pydantic_model(self.Parameters, "player_config")
        self.create_subscription(
            PlaySound,
            "play_sound",
            self.on_play_sound,
            qos_profile=qos_profile_services_default,
        )

    def on_play_sound(self, msg: PlaySound) -> None:
        file_path = self.params.sound_effects_directory / msg.sound_filename
        if not file_path.exists():
            raise FileNotFoundError(
                f"The specified filename '{msg.sound_filename}' does"
                " not exist in the configured sound directory: "
                f"{self.params.sound_effects_directory}"
            )

        command = ["paplay", str(file_path)]

        logging.info(f"Playing sound using command: {command}")
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError:
            logging.exception(
                "Sound Player encountered error while attempting to play sound"
            )


main = create_spin_function(SoundPlayer)
