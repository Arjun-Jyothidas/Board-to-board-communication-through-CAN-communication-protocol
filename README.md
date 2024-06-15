# CAN-communication-protocol-between-two-STM32-boards-in-Normal-mode
Following repository contains HAL C codes for developing a communication between two boards - a board, transmitting the message and a board, receiving the message through Controller Area Network (CAN) communication protocol.
Only the main source files are added in this repository as the rest of the files are generated automatically by the STMCubeIDE.
Both the main .c files for respective transmitter and receiver boards are included in the repository.
Also the respective Tx Hearder configurations and CAN filter configurations are configured in the main code itself.
The target microcontroller - STM32U545RE-Q is selected in the STMCubeMX, matching the hardware.
Hardware connection also requires two SN65HVD230 CAN transceivers conected through two wires twisted to form differential signals - CAN high and CAN low.
The following repository shows that master sends the transmitting data to the slave through CAN communication.
