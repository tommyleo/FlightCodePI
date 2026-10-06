# ELRS binding command

The Receiver tab exposes **Bind receiver** only when ELRS is selected.
Apply the protocol and port before binding. This requires ExpressLRS 3.4+
and both UART wires: receiver TX to FC RX, FC TX to receiver RX.
FlightCodePI uses GP0 for RX and GP1 for TX. FlightCode uses the TX pin
of the configured receiver UART (CLRacing F4: RX4 PA1 / TX4 PA0).

The configurator enables the button only when the firmware advertises
`RECEIVER_BIND`. `BIND_RECEIVER` is rejected while armed, during motor
testing/PID simulation, or when the applied protocol is not ELRS/CRSF.
The firmware replies `@CFG OK BIND_RECEIVER` after sending the nine-byte
CRSF command, or `@CFG ERROR BIND_TX_FAILED` if transmission fails.
No receiver acknowledgement is available: check the LED and channels,
and select Bind on the transmitter to complete pairing.

TX is temporarily enabled for the command and then returned to input.
Incoming channels stay enabled. This does not add periodic CRSF telemetry.

Reference: https://www.expresslrs.org/quick-start/binding/

Validation: `tools/test-elrs-bind.ps1 -Compiler gcc` checks both CRCs and
command guards against the actual protocol handler. The FlightCode Foxeer
UART tests additionally check TX routing, RX preservation and TX cleanup
on success and timeout. Hardware binding still needs a powered receiver.
