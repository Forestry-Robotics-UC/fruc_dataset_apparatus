import rospy, os
from rostopic import ROSTopicHz
from std_msgs.msg import Header
from diagnostic_msgs.msg import KeyValue, DiagnosticStatus, DiagnosticArray


class MonitorSystem():

    def __init__(self):
        """
            Construtor Function, initializes all the variables used in the MonitorSystem class Object
            Inputs:
                - self
            Outputs:
                - None
        """
        #Initialize variables
        window_size = -1
        self.rt = ROSTopicHz(window_size, filter_expr=None)
        self.rate = 0.5 #0.5Hz
        
        #Read the TOPICS enviroment variable
        topics_env = os.getenv("TOPICS")

        #Saves the topics as an array
        self.topic_array = topics_env.strip().split("\n") 
        
        self.rt_array = []
        
        # Create a ROSTopicHz object for every topic and adds them to the rt_array array
        for topic in self.topic_array:
            rt = ROSTopicHz(window_size, filter_expr=None)
            self.rt_array.append(rt)
            rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz)
        
        #Create the publisher object
        self.pub = rospy.Publisher("/diagnostic/system", DiagnosticArray, queue_size=10)

    def get_temperatures(self):
        """
            It returns the DiagnosticStatus message that contains all temperatures available 
            in the host computer. The temperatures are obtained by reading the corresponding
            Linux files. This function will be called in the function run() to construct the DiagnosticArray 
            message that will be publish.
            Inputs:
                - self
            Outputs:
                - ds: ROS::DiagnosticStatus message containing all temperatures in the system
        """
        kv_array = []
        count = 0

        ds = DiagnosticStatus()
        
        #Loop through every temperature file in the host computer
        while(True):
            try:
                with open('/sys/class/thermal/thermal_zone{}/temp'.format(count), 'r') as f:
                    temp = float(f.read()) / 1000.0 #The temperature is in millis

                    #Generate the KeyValue message
                    kv = KeyValue()
                    kv.key = 'temp{}'.format(count)
                    kv.value = str(temp)
                    kv_array.append(kv)
                    count+=1 #Increase the counting
            except:
                #If the file does not exist it means that the computer has count-1 sensors
                break

        #Construct the DiagnosticStatus Message
        ds.name = "Computer Temperatures (°C)"
        ds.message = ""
        ds.hardware_id = "udoo_bolt"
        ds.values = kv_array

        #TODO: Add the level operation
        ds.level = 0

        return ds
    
    def get_cpu_frequencies(self):
        """
            It returns the DiagnosticStatus message that contains all the current CPU frequencies
            and the maximum frequency. The frequencies are obtained by reading the corresponding
            Linux files. This function will be called in the function run() to construct the DiagnosticArray 
            message that will be publish.
            Inputs:
                - self
            Outputs:
                - ds: ROS::DiagnosticStatus message containing all core frequencies and max frequency
        """
        count = 0
        kv_array = []
        ds = DiagnosticStatus()
        
        #Get the maximum frequency of the CPU
        with open('/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq', 'r') as f:
            max_freq = float(f.read())/1000000.0 #Store frequencies in MHz
            kv = KeyValue()
            kv.key = 'Max freq'
            kv.value = str(max_freq)
            kv_array.append(kv)

        #Get the current frequencies of all cores
        while True:
            try:
                with open('/sys/devices/system/cpu/cpu{}/cpufreq/cpuinfo_cur_freq'.format(count), 'r') as f:
                    freq = float(f.read())/1000000.0 #Store frequencies in MHz
                    kv = KeyValue()
                    kv.key = 'cpu{}'.format(count)
                    kv.value = str(freq)
                    kv_array.append(kv)
                    count +=1
            except:
                #If the file does not exist it means that the computer has count-1 cores
                break
        
        ds.name = "CPU Frequencies (MHz)"
        ds.message = ""
        ds.hardware_id = "udoo_bolt"
        ds.values = kv_array
        ds.level = 0

        return ds

    def get_topic_freq(self):
        """
            It returns the DiagnosticStatus message that contains all the topic frequencies
            This function will be called in the function run() to construct the DiagnosticArray 
            message that will be publish.
            Inputs:
                - self
            Outputs:
                - ds: ROS::DiagnosticStatus message containing all topic frequencies
        """
        kv_array = []
        ds = DiagnosticStatus()

        for i in range(0,len(self.topic_array)):
            kv = KeyValue()
            kv.key = self.topic_array[i] +' frequency'
            kv.value = str(self.get_hz(self.rt_array[i]))
            kv_array.append(kv)
        
        ds.name = "Topic Frequencies (Hz)"
        ds.message = ""
        ds.hardware_id = "sensors"
        ds.values = kv_array
        ds.level = 0

        return ds

    def get_hz(self, rt):
        """
            This function computes the frequency rate of the provided topic. It is an adaptation of
            the function print_hz() from the ROSTopicHz ROS's class.
            Inputs:
                - self
                - rt: ROS::ROSTopicHz object
            Outputs:
                - rate: float representing the frequency rate. If rate == -1, the true rate could not be computed
        """
        kv_array = []
        if not rt.times: 
            return -1
        elif rt.msg_tn == rt.last_printed_tn: 
            return -1 
        with rt.lock:
            n = len(rt.times)
            mean = sum(rt.times)/n
            rate = 1./mean if mean > 0. else 0

            rt.last_printed_tn = rt.msg_tn

            return rate

    def run(self):
        """
            This is the main function that will only stops running when the ROS master stops.
            This function calls all the get_* function to assemble a DiagnosticArray() message and publish
            it in the ROS channel.
            Inputs:
                - self
            Outputs:
                - None
        """
        da = DiagnosticArray()
        da.header = Header()
        
        #Keep alive if the ROS master is running
        while( not rospy.is_shutdown()):
            
            #get the relevant informations
            temp_status = self.get_temperatures()
            cpu_freq_status = self.get_cpu_frequencies()
            topic_freq_status = self.get_topic_freq()
            
            #Assemble the DiagnosticArray message
            da.status = [temp_status, cpu_freq_status, topic_freq_status]
            da.header.stamp = rospy.Time.now()

            #Publish the DiagnosticArray message
            self.pub.publish(da)
            rospy.sleep(self.rate)

if __name__ == '__main__':
    try:
        rospy.init_node("system_publisher")
        ms = MonitorSystem()
        ms.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
