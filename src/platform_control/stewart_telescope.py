from src.telescope.telescope import Telescope
import yaml

class StewartTelescope(Telescope):
    """
        config must contain:
    """
    def __init__(self, config):
        telescope_config = self.unload_config(config)
        super().__init__(telescope_config)

    """
        given a path to a telescope config file, it loads the file
        and sets all necessary items
    """
    def unload_config(self, path):
        with open(
            path,
            "r"
        ) as f:
            config = yaml.safe_load(f)
        return config

    def rotate_panel(self, panel_id, rotation):
        return super().rotate_panel(panel_id, rotation)

    def reset(self):
        return super().reset()

    def update(self):
        return super().update(img_centroid_locations, centroid_type)