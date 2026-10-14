# Verification status

| Item | Status | Verification command |
|---|---|---|
| Host compiler and Make | VERIFIED by command output | `g++ --version`, `make --version` |
| LADRC identification fit | VERIFIED by test | `make test` |
| Safety checks | VERIFIED by test | `make test` |
| Bezier endpoints and curvature basics | VERIFIED by test | `make test` |
| Bezier sample-count convergence and infeasible-case coverage | NOT VERIFIED | Tests not implemented |
| S-curve limits and timing | VERIFIED by test | `make test` |
| LADRC tracking and saturation | VERIFIED by test | `make test` |
| LADRC 20% settling-time criterion and disturbance tolerance | NOT VERIFIED | Tests not implemented |
| Drive saturation freeze and measured-pose projection | VERIFIED by test | `make test` |
| Full unicycle motor-delay integration threshold | NOT VERIFIED | No host simulation implemented |
| Full VEX project link | VERIFIED by build output | `make -f makefile OS=Windows_NT VEX_SDK_PATH=... V=1` |
| Physical motor/sensor calibration | NOT VERIFIED | Robot measurement required |
