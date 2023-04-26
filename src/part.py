from typing import List


class Part:
    def __init__(self, assembly):
        self.body = assembly.get_component_by_name('part')
        pass

    def get_trajectory(self) -> List[List]:
        pass
