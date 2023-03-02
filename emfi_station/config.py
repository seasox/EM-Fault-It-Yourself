# EMFI Station - Orchestrate electromagnetic fault injection attacks
# Copyright (C) 2022 Niclas Kühnapfel
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
from dataclasses import dataclass

@dataclass
class Config:
    """
    Encapsulates configuration variables
    """
    host: str
    http_port: int
    simulate: bool
    attack_dir: str
    log_dir: str
    world_cam: tuple[str, str, int]
    positioning_cam: tuple[str, str, int]
    calibration_cam: tuple[str, str, int]
    marlin: tuple[str, str, int]
    safe_z: int
