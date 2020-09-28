from i40.loaders.hardware_loader import HardwareLoader
from data_handling.wrs_persistence import WRSPersistence
from i40.loaders.cell_loader import CellLoader


def get_gripper():
    persistence = WRSPersistence.driver()
    cell = CellLoader.load(persistence)
    gripper_a_model = cell.find_tree_device("gripper_a")
    gripper_a = HardwareLoader.get_gripper(gripper_a_model, persistence)
    gripper_a.activate()
    return gripper_a
