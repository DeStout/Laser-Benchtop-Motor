if __name__ == '__main__':
    import os
    import time
    import sys

    # https://github.com/MSLNZ/msl-equipment
    # pip install https://github.com/MSLNZ/msl-equipment/archive/master.zip
    from msl.equipment.resources.thorlabs import MotionControl
    from msl.equipment import EquipmentRecord, ConnectionRecord, Backend

    # ensure that the Kinesis folder is available on PATH
    # Kinesis must be installed from https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=10285
    # Point kinesis_path to Kinesis installation location
    kinesis_path = 'C:/Program Files/Thorlabs/Kinesis'
    os.environ['PATH'] += os.pathsep + kinesis_path

    # Set the parameters for the individual channels of the controller
    channel_1 = {
        "serial_no" : 1,
        "poll_rate" : 200,
        "min_velocity" : 0,
        "homing_velocity" : 1,
        "feed_velocity": 2,
        "rapid_velocity" : 30,
        "acceleration" : 30
    }
    channel_2 = {
        "serial_no" : 2,
        "poll_rate" : 200,
        "min_velocity" : 0,
        "homing_velocity" : 1,
        "feed_velocity": 2,
        "rapid_velocity" : 30,
        "acceleration" : 30
    }
    channels = [channel_1, channel_2]


    # https://msl-equipment.readthedocs.io/en/latest/_modules/msl/equipment/resources/thorlabs/kinesis/motion_control.html#MotionControl.build_device_list
    # Must be called before connecting to the controller
    # Builds a list of all connected devices that haven't been opened yet
    print('Building the device list...')
    MotionControl.build_device_list()
    if MotionControl.get_device_list_size() == 0:
        print('There are no devices in the device list')
        sys.exit(0)

    # Define the controller we expect to connect to
    record = EquipmentRecord(
        manufacturer='Thorlabs',
        model='BSC203',
        # All 3 channel benchtop stepper motor serial number begin with 70
        # Serial number printed on the rear of the controller
        serial='70150504',
        connection=ConnectionRecord(
            address='SDK::Thorlabs.MotionControl.Benchtop.StepperMotor.dll',
            backend=Backend.MSL,
        )
    )


    # Iterate through the channels previously defined
    # Begin polling each, load the setting of each, and convert and set the defined velocity parameters of each
    # Finishes by homing a channel if it's needed
    def setup_channels():
        for channel in channels:
            # https://msl-equipment.readthedocs.io/en/latest/_modules/msl/equipment/resources/thorlabs/kinesis/benchtop_stepper_motor.html#BenchtopStepperMotor.start_polling
            # Continually requests status and position messages
            motor.start_polling(channel["serial_no"], channel["poll_rate"])
            try:
                # Sometimes throws a windows error but is hopefully a pycharm issue
                motor.load_settings(channel["serial_no"])
            except:
                print("Unable to load settings")
                sys.exit()
            time.sleep(0.1)

            # Load_settings must be called before converting units or 0 will be returned
            homing_velocity = motor.get_device_unit_from_real_value(channel["serial_no"], channel["homing_velocity"], 1)
            min_velocity = motor.get_device_unit_from_real_value(channel["serial_no"], channel["min_velocity"], 1)
            max_velocity = motor.get_device_unit_from_real_value(channel["serial_no"], channel["feed_velocity"], 1)
            acceleration = motor.get_device_unit_from_real_value(channel["serial_no"], channel["acceleration"], 2)

            motor.set_homing_velocity(channel["serial_no"], homing_velocity)
            motor.set_vel_params_block(channel["serial_no"], min_velocity, max_velocity, acceleration)

            # Motor homing parameters can be changed
            #https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.html#msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.BenchtopStepperMotor.set_homing_params_block
            if not motor.can_move_without_homing_first(channel["serial_no"]):
                motor.home(channel["serial_no"])
                wait(0, channel["serial_no"])
                print("Channel {} Successfully Homed".format(channel["serial_no"]))


    def do_raster():
        # Raster path definitions in real world values (mm)
        # Negative travel lengths will move towards the default homing position
        # Start_positions must be positive (motor has range 0 - 100)
        raster_direction = 'X'
        raster_x_start_position = 10
        raster_y_start_position = 10
        raster_path_length = -5
        raster_total_step_length = -5
        raster_num_steps = 20
        raster_step_length = raster_total_step_length / raster_num_steps

        # Setup the axes to step and travel along depending on 'raster_direction'
        # Checks if desired raster path stays within motor travel limits
        if raster_direction == 'X':
            path_axis = channel_1["serial_no"]
            step_axis = channel_2["serial_no"]
            path_start_position = motor.get_device_unit_from_real_value(path_axis, raster_x_start_position, 0)
            step_start_position = motor.get_device_unit_from_real_value(step_axis, raster_y_start_position, 0)
            path_feed_velocity = motor.get_device_unit_from_real_value(path_axis, channel_1["feed_velocity"], 1)
            step_feed_velocity = motor.get_device_unit_from_real_value(step_axis, channel_2["feed_velocity"], 1)
            path_rapid_velocity = motor.get_device_unit_from_real_value(path_axis, channel_1["rapid_velocity"], 1)
            step_rapid_velocity = motor.get_device_unit_from_real_value(step_axis, channel_2["rapid_velocity"], 1)
            path_acceleration = motor.get_device_unit_from_real_value(path_axis, channel_1["acceleration"], 2)
            step_acceleration = motor.get_device_unit_from_real_value(step_axis, channel_2["acceleration"], 2)

            if not 0 <= raster_x_start_position + raster_path_length <= 100:
                print("Raster path axis out of bounds: ", raster_x_start_position + raster_path_length)
                return
            elif not 0 <= raster_y_start_position + (raster_step_length * raster_num_steps) <= 100:
                print("Raster step axis out of bounds: ", raster_y_start_position + (raster_step_length * raster_num_steps))
                return
        elif raster_direction == 'Y':
            path_axis = channel_2["serial_no"]
            step_axis = channel_1["serial_no"]
            path_start_position = motor.get_device_unit_from_real_value(path_axis, raster_y_start_position, 0)
            step_start_position = motor.get_device_unit_from_real_value(step_axis, raster_x_start_position, 0)
            path_feed_velocity = motor.get_device_unit_from_real_value(path_axis, channel_2["feed_velocity"], 1)
            step_feed_velocity = motor.get_device_unit_from_real_value(step_axis, channel_1["feed_velocity"], 1)
            path_rapid_velocity = motor.get_device_unit_from_real_value(path_axis, channel_2["rapid_velocity"], 1)
            step_rapid_velocity = motor.get_device_unit_from_real_value(step_axis, channel_1["rapid_velocity"], 1)
            path_acceleration = motor.get_device_unit_from_real_value(path_axis, channel_2["acceleration"], 2)
            step_acceleration = motor.get_device_unit_from_real_value(step_axis, channel_1["acceleration"], 2)

            if not 0 <= raster_y_start_position + raster_path_length <= 100:
                print("Raster path axis out of bounds: ", raster_y_start_position + raster_path_length)
                return
            elif not 0 <= raster_x_start_position + (raster_step_length * raster_num_steps) <= 100:
                print("Raster step axis out of bounds: ",
                      raster_x_start_position + (raster_step_length * raster_num_steps))
                return
        else:
            print("Invalid raster_direction.")
            sys.exit()

        path_length = motor.get_device_unit_from_real_value(path_axis, raster_path_length, 0)
        step_length = motor.get_device_unit_from_real_value(step_axis, raster_step_length, 0)

        if motor.can_move_without_homing_first(path_axis) and motor.can_move_without_homing_first(step_axis):
            # Move to starting position with rapid velocity
            motor.set_vel_params(path_axis, path_rapid_velocity, path_acceleration)
            motor.set_vel_params(step_axis, step_rapid_velocity, step_acceleration)

            print("Beginning Raster...")
            # 'Move_to_position' moves to a discrete position in device units
            # Can also set position with 'set_move_absolute_position' and move with 'move_absolute'
            # https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.html#msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.BenchtopStepperMotor.set_move_absolute_position
            # https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.html#msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.BenchtopStepperMotor.move_absolute
            motor.move_to_position(path_axis, path_start_position)
            wait(1, path_axis)
            motor.move_to_position(step_axis, step_start_position)
            wait(1, step_axis)

            # Do raster path at feed velocity
            motor.set_vel_params(path_axis, path_feed_velocity, path_acceleration)
            motor.set_vel_params(step_axis, step_feed_velocity, step_acceleration)

            for step in range(raster_num_steps + 1):
                # Can also set relative distance with 'set_move_relative_distance' and move with 'move_relative_distance'
                # https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.html#msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.BenchtopStepperMotor.set_move_relative_distance
                # https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.html#msl.equipment.resources.thorlabs.kinesis.benchtop_stepper_motor.BenchtopStepperMotor.move_relative_distance
                motor.move_relative(path_axis, path_length)
                wait(1, path_axis)
                path_length *= -1

                if step == raster_num_steps:
                    break

                motor.move_relative(step_axis, step_length)
                wait(1, step_axis)

            print("...Raster Completed")
            time.sleep(2)

            # Reset to rapid velocity to return to starting position
            motor.set_vel_params(path_axis, path_rapid_velocity, path_acceleration)
            motor.set_vel_params(step_axis, step_rapid_velocity, step_acceleration)

            print("Returning to Start Position")
            motor.move_to_position(path_axis, path_start_position)
            wait(1, path_axis)
            motor.move_to_position(step_axis, step_start_position)
            wait(1, step_axis)


    # Pauses program execution until the controller returns a desired message value
    # Message return value varies and depends on motor functionality being performed
    # https://msl-equipment.readthedocs.io/en/latest/_api/msl.equipment.resources.thorlabs.kinesis.messages.html#module-msl.equipment.resources.thorlabs.kinesis.messages
    def wait(value, channel):
        motor.clear_message_queue(channel)
        message_type, message_id, _ = motor.wait_for_message(channel)
        print(message_type, message_id)
        while message_type != 2 or message_id != value:
            # position = motor.get_position(channel)
            # print('  Channel {} at position {} [device units]'.format(channel, position))
            message_type, message_id, _ = motor.wait_for_message(channel)
            print(message_type, message_id)

    # Connect to and open the Benchtop Stepper Motor
    # Device list must be created beforehand
    motor = record.connect()
    motor.open()
    print('Connected to {}'.format(motor))
    time.sleep(1)

    setup_channels()
    do_raster()
