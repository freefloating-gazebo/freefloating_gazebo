#!/usr/bin/env python

'''
Use rqt_plot to plot various topic types

author: Olivier Kermorgant <kermorgant@unistra.fr>
'''

# ROS
import rospy
# Other tools
from commands import getoutput
import os, argparse


if __name__ == '__main__':
    '''
    Begin of main code
    '''

    # Parse arguments
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.description = 'A script to allow rqt_plot to plot various topic types'
    parser.add_argument('topic', metavar='topic', type=str, help='Topic to be plotted')
    args = parser.parse_args()
    
    # name of the node
    rospy.init_node('plot_topic')

    # get topic type
    topic_struct = args.topic.split('/') + ['']
    msg_type = msg_attr = topic_actual = ''
    i = 1
    while i < len(topic_struct)-1:
        ros_out = getoutput('rostopic info %s' % '/'.join(topic_struct[:-i])).splitlines()[0].split(' ')
        if ros_out[0] == 'ERROR:':
            # actually not a topic
            i += 1
        else:
            # found a topic
            msg_type = ros_out[-1]
            msg_attr = '/'.join(topic_struct[-i:-1])
            topic_actual = '/'.join(topic_struct[:-i])
            break
            
    '''
    Parse msg_type to build a dictionary until Float or Float[] are found
    Ignore fields given in ignore    
    '''
    ignore = ['header', 'covariance', 'layout']
 
    # Read message structure
    ros_out = getoutput('rosmsg show %s' % msg_type).splitlines()
    ros_out.reverse()

    # Recursive fct to get the tree of a variable
    def GetMsgTree(ind):
        new_elem = ros_out[ind].lstrip().split(' ')[-1]
        for i,line in enumerate(ros_out[ind+1:]):
            if (len(line) - len(line.lstrip())) < (len(ros_out[ind]) - len(ros_out[ind].lstrip())):     # compare leading spaces
                return GetMsgTree(ind+i+1) + '/' + new_elem
        return new_elem

    msg_floats = []
    msg_arrays = []
    msg_map = {'float': msg_floats, 'array': msg_arrays}
    
    for i,line in enumerate(ros_out):
        elem_type = ''
        if 'float64[]' in line or 'float32[]' in line:
            elem_type = 'array'
        elif 'float64' in line or 'float32' in line:
            elem_type = 'float'
            
        if elem_type != '':
            # get new element
            new_elem = GetMsgTree(i)
            
            if msg_attr != '':
                if msg_attr in new_elem:                                    # if new_elem in the desired scope, add it
                    msg_map[elem_type].insert(0, new_elem)
            elif [ign in new_elem for ign in ignore].count(True) == 0:      # add it also if not in ignore list
                    msg_map[elem_type].insert(0, new_elem)
                
    # if arrays are to be plotted, wait for message in order to get the dimensions
    msg_array_dim = []
    if len(msg_arrays) != 0:
        # importe module and message class
        package_module = __import__(msg_type.split('/')[0] + '.msg')
        msg_class = getattr(getattr(package_module, 'msg'), msg_type.split('/')[1])
        
        # wait for message
        msg = rospy.wait_for_message(topic_actual, msg_class)
        # parse message
        for i,msg_array in enumerate(msg_arrays):
            attr = msg
            for sub_msg in msg_array.split('/'):
                attr = getattr(attr, sub_msg)
            msg_array_dim.append(len(attr))
                        
    # get list of all values to be plotted
    all_topics = []
    if len(msg_floats) != 0:
        all_topics += ['%s/%s' % (topic_actual, msg_float) for msg_float in msg_floats]
    if len(msg_arrays) != 0:
        for i,msg_array in enumerate(msg_arrays):
            all_topics += ['%s/%s[%i]' % (topic_actual, msg_array, j) for j in xrange(msg_array_dim[i])]
            
    # separate in several subplots according to message attributes
    # TODO: does not seem to change anything, was working in rxplot
    sub_topics = [all_topics[0]]
    topic_key = all_topics[0].split('/')[:-1]
    for topic in all_topics[1:]:
        if topic.split('/')[:-1] == topic_key:
            sub_topics[-1] += ',' + topic
        else:
            topic_key = topic.split('/')[:-1]
            sub_topics.append(topic)
        
    # build command line
    cmd_line = 'rqt_plot ' + ' '.join(sub_topics)

    # launch rqt_plot
    print 'Executing', cmd_line
    os.system(cmd_line)
