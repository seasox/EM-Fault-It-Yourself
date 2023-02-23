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

import logging
from emfi_station import Attack


class ToyAttack(Attack):

    def __init__(self):
        super().__init__(start_pos=None, end_pos=None, step_size=None, max_target_temp=40, cooling=1)

    @staticmethod
    def name() -> str:
        """
        Returns name of target/attack.
        :return: Attack name
        """
        return 'Toy Attack'

    def init(self) -> None:
        """
        Initialize hardware and define e.g. ChipShouter settings.
        :return: None
        """
        return None

    def shout(self) -> None:
        """
        Shout procedure.
        :return: None
        """
        return None

    def was_successful(self) -> bool:
        """
        Determines if attack was successful or not.
        Returns True if successful.
        :return: True or False
        """
        return False

    def reset_target(self) -> None:
        """
        Resets the target after an attack.
        :return: None
        """
        return None

    def critical_check(self) -> bool:
        """
        Runs critical checks after every shout. May wait some time.
        Returns False if check failed which leads to a shutdown.
        :return: True or False
        """
        return True
