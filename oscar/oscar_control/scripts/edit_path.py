#!/usr/bin/env python
import logging, sys, os, cv2, numpy as np, rospy, tf, std_msgs.msg, geometry_msgs.msg, nav_msgs.msg
import gi, threading
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib, GObject
import pdb
import guidance, utils


class GUI:
    def __init__(self):
        self.b = Gtk.Builder()
        gui_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_editor_gui.xml')
        self.b.add_from_file(gui_xml_path)
        self.window = self.b.get_object("window")
        #self.pos_entries = [self.b.get_object("entry_pos_"+axis) for axis in ['x', 'y', 'z']]
        #self.ori_entries = [self.b.get_object("entry_ori_"+axis) for axis in ['r', 'p', 'y']]
        self.window.show_all()

    def display_path(self, _p):
        buf = self.b.get_object("textview1").get_buffer()
        buf.set_text("hello")

    def request_path(self, action):
        dialog = Gtk.FileChooserDialog("Please choose a file", self.window, action,
                                       (Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
                                        Gtk.STOCK_OPEN, Gtk.ResponseType.OK))
        ret = dialog.run()
        file_path = dialog.get_filename() if ret == Gtk.ResponseType.OK else None
        dialog.destroy()
        return file_path


        
    def run(self):
        Gtk.main()
        

class PathEditor:
    def __init__(self, **kwargs):
        self.pub_path = rospy.Publisher('edit_path/path', nav_msgs.msg.Path, queue_size=1) 
        self.smocap_listener = utils.SmocapListener()
        self.rate = rospy.Rate(10.)
        self._path = guidance.path.Path(**kwargs)
        self.save = kwargs.get('save', None)
        rospy.Subscriber("/edit_path/mode", std_msgs.msg.String, self.on_mode)
        rospy.Subscriber("/edit_path/goal", geometry_msgs.msg.PoseStamped, self.on_goal)
        self.set_mode(kwargs.get('mode', 'edit'))


    def load_path(self, filename):
        self._path.load(filename)

    def save_path(self, filename):
        self._path.save(filename)
   
    def set_mode(self, mode):
        self.mode = mode
        print('setting mode to {}'.format(self.mode))
        
        
    def publish_path(self):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for l, y in zip(self._path.points, self._path.headings):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = tf.transformations.quaternion_from_euler(*[0, 0, y])
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

    def on_goal(self, msg):
        xy = utils.array_of_xyz(msg.pose.position)[:2]
        psi = tf.transformations.euler_from_quaternion(utils.list_of_xyzw(msg.pose.orientation))[2]
        
        if self.mode == "record":
            self._path.append_points((xy, psi))
        elif self.mode == "edit":
            i, p = self._path.find_closest(xy)
            print("editing pt {} from {} to {}".format(i, p, xy))
            self._path.move_point(i, xy, psi)
            self._path.reset()
        elif self.mode == "insert":
            i, p = self._path.find_closest(xy)
            print("inserting pt {} at {}".format(xy, i))
            self._path.insert_point(i, xy, psi)
            
    def on_mode(self, msg):
        #p0, psi = self.smocap_listener.get_loc_and_yaw()
        self.set_mode(msg.data)
        
    def run(self):
        while not rospy.is_shutdown():
            self.publish_path()
            self.rate.sleep()
        if self.save is not None: self._path.save(self.save)


class App:
    def __init__(self):
        self.node = PathEditor()
        self.gui = GUI()
        self.register_gui()

    def register_gui(self):
        self.gui.window.connect("delete-event", self.quit)
        for i,s in enumerate(['new', 'open', 'save', 'save_as']):
            item = self.gui.b.get_object("imagemenuitem{}".format(i+1))
            item.connect("activate", self.callback, s)

    def callback(self, emitter, action):
        #print emitter, action
        if action == 'open':
            filename = self.gui.request_path(Gtk.FileChooserAction.OPEN)
            if filename is not None:
                self.node.load_path(filename)
        if action == 'save_as':
            filename = self.gui.request_path(Gtk.FileChooserAction.SAVE)
            if filename is not None:
                self.node.save_path(filename)
                                             
    def run(self):
        self.ros_thread = threading.Thread(target=self.node.run)
        self.ros_thread.start()
        self.gui.run()

    def quit(self, a, b):
        rospy.signal_shutdown("because the world is blue")
        self.ros_thread.join()
        print 'ros thread ended'
        Gtk.main_quit()

        
def main(args):
    #PathEditor(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz',
    #           save='/tmp/foo',
    #           mode='edit').run()
    App().run()

if __name__ == '__main__':
    rospy.init_node('record_path')
    main(sys.argv)
