#!/usr/bin/env python3
import rospy
import subprocess
import rospkg
import os
import signal

# from https://gist.github.com/marco-tranzatto/8be49b81b1ab371dcb5d4e350365398a

class RosbagRecord:
    def __init__(self):
        
        conf_file_path = os.path.join(rospkg.RosPack().get_path('quad_ufabc'), 'bags/')     
        script_file_path = os.path.join(rospkg.RosPack().get_path('quad_ufabc'), 'scripts/rosbag_script')     
        
        rospy.set_param('~record_script', script_file_path)
        rospy.set_param('~record_folder', conf_file_path)
        #rospy.set_param('~record_script', './rosbag_script')
        #rospy.set_param('~record_folder', '../bags')

        if rospy.has_param('~record_script') and rospy.has_param('~record_folder'):
            self.record_script = rospy.get_param('~record_script')
            self.record_folder = rospy.get_param('~record_folder')
            rospy.on_shutdown(self.stop_recording_handler)

            # Start recording.
            command = "source " + self.record_script
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True,cwd=self.record_folder, executable='/bin/bash')

            # Wait for shutdown signal to close rosbag record
            rospy.spin()
        else:
            rospy.signal_shutdown(rospy.get_name() + ' no record script or folder specified.')

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)

    def stop_recording_handler(self):
        rospy.loginfo(rospy.get_name() + ' stop recording.')
        self.terminate_ros_node("/record")

if __name__ == '__main__':
    rospy.init_node('rosbag_record')
    rospy.loginfo(rospy.get_name() + ' start')
    
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rosbag_record = RosbagRecord()
    except rospy.ROSInterruptException:
        pass