# SPDX-License-Identifier: CC0-1.0

cam_capture = files('capture-test.sh')

test('cam-capture-test', cam_capture,
    args : cam,
    suite : 'integration',
    is_parallel : false,
    #protocol : 'tap',
    timeout : 60)
