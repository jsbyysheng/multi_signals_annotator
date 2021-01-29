# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os
import time

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import qDebug, QFileInfo, Qt, qWarning, Signal
from python_qt_binding.QtGui import QIcon, QResizeEvent, QStandardItem, QStandardItemModel
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsView, QWidget, QMessageBox

import rosbag
from rqt_bag import bag_helper
from .bag_timeline import BagTimeline
from .topic_selection import TopicSelection

class BagGraphicsView(QGraphicsView):

    def __init__(self, parent=None):
        super(BagGraphicsView, self).__init__()


class BagWidget(QWidget):

    """
    Widget for use with Bag class to display and replay bag files
    Handles all widget callbacks and contains the instance of BagTimeline for storing visualizing bag data
    """

    last_open_dir = os.getcwd()
    last_open_saving_dir = os.getcwd()
    set_status_text = Signal(str)

    def __init__(self, context, publish_clock):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(BagWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_bag'), 'resource', 'bag_widget.ui')
        loadUi(ui_file, self, {'BagGraphicsView': BagGraphicsView})

        self.setObjectName('BagWidget')

        self._timeline = BagTimeline(context, publish_clock)
        self.graphics_view.setScene(self._timeline)

        self.graphics_view.resizeEvent = self._resizeEvent
        self.graphics_view.setMouseTracking(True)

        self.topicsListView.resizeEvent = self._resizeEvent
        self.topicsListViewModel = QStandardItemModel()

        self.play_icon = QIcon.fromTheme('media-playback-start')
        self.pause_icon = QIcon.fromTheme('media-playback-pause')
        self.play_button.setIcon(self.play_icon)
        self.begin_button.setIcon(QIcon.fromTheme('media-skip-backward'))
        self.end_button.setIcon(QIcon.fromTheme('media-skip-forward'))
        self.slower_button.setIcon(QIcon.fromTheme('media-seek-backward'))
        self.faster_button.setIcon(QIcon.fromTheme('media-seek-forward'))
        self.previous_button.setIcon(QIcon.fromTheme('go-previous'))
        self.next_button.setIcon(QIcon.fromTheme('go-next'))
        self.zoom_in_button.setIcon(QIcon.fromTheme('zoom-in'))
        self.zoom_out_button.setIcon(QIcon.fromTheme('zoom-out'))
        self.zoom_all_button.setIcon(QIcon.fromTheme('zoom-original'))
        self.thumbs_button.setIcon(QIcon.fromTheme('insert-image'))

        self.play_button.clicked[bool].connect(self._handle_play_clicked)
        self.thumbs_button.clicked[bool].connect(self._handle_thumbs_clicked)
        self.zoom_in_button.clicked[bool].connect(self._handle_zoom_in_clicked)
        self.zoom_out_button.clicked[bool].connect(self._handle_zoom_out_clicked)
        self.zoom_all_button.clicked[bool].connect(self._handle_zoom_all_clicked)
        self.previous_button.clicked[bool].connect(self._handle_previous_clicked)
        self.next_button.clicked[bool].connect(self._handle_next_clicked)
        self.faster_button.clicked[bool].connect(self._handle_faster_clicked)
        self.slower_button.clicked[bool].connect(self._handle_slower_clicked)
        self.begin_button.clicked[bool].connect(self._handle_begin_clicked)
        self.end_button.clicked[bool].connect(self._handle_end_clicked)
        self.pushButton_load.clicked[bool].connect(self._handle_load_clicked)
        self.pushButton_export.clicked[bool].connect(self._handle_save_clicked)
        self.graphics_view.mousePressEvent = self._timeline.on_mouse_down
        self.graphics_view.mouseReleaseEvent = self._timeline.on_mouse_up
        self.graphics_view.mouseMoveEvent = self._timeline.on_mouse_move
        self.graphics_view.wheelEvent = self._timeline.on_mousewheel
        self.closeEvent = self.handle_close
        self.keyPressEvent = self.on_key_press

        self.topicsListView.clicked.connect(self._handle_topicsList_clicked)

        # TODO when the closeEvent is properly called by ROS_GUI implement that
        # event instead of destroyed
        self.destroyed.connect(self.handle_destroy)

        self.graphics_view.keyPressEvent = self.graphics_view_on_key_press
        self.play_button.setEnabled(False)
        self.thumbs_button.setEnabled(False)
        self.zoom_in_button.setEnabled(False)
        self.zoom_out_button.setEnabled(False)
        self.zoom_all_button.setEnabled(False)
        self.previous_button.setEnabled(False)
        self.next_button.setEnabled(False)
        self.faster_button.setEnabled(False)
        self.slower_button.setEnabled(False)
        self.begin_button.setEnabled(False)
        self.end_button.setEnabled(False)
        self.pushButton_export.setEnabled(False)

        self.lineEdit_bag_file_path.setText(self.last_open_dir)
        self.lineEdit_export_path.setText(self.last_open_saving_dir)

        self._timeline.status_bar_changed_signal.connect(self._update_status_bar)
        self._timeline.set_status_text.connect(self._set_status_text)
        self.set_status_text.connect(self._set_status_text)

        self.bagfile_name = None

    def graphics_view_on_key_press(self, event):
        key = event.key()
        if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down, Qt.Key_PageUp, Qt.Key_PageDown):
            # This causes the graphics view to ignore these keys so they can be caught
            # by the bag_widget keyPressEvent
            event.ignore()
        else:
            # Maintains functionality for all other keys QGraphicsView implements
            QGraphicsView.keyPressEvent(self.graphics_view, event)

    # callbacks for ui events
    def on_key_press(self, event):
        key = event.key()
        if key == Qt.Key_Space:
            self._timeline.toggle_play()
        elif key == Qt.Key_Home:
            self._timeline.navigate_start()
        elif key == Qt.Key_End:
            self._handle_end_clicked()
        elif key == Qt.Key_Plus or key == Qt.Key_Equal:
            self._handle_faster_clicked()
        elif key == Qt.Key_Minus:
            self._handle_slower_clicked()
        elif key == Qt.Key_Left:
            self._timeline.translate_timeline_left()
        elif key == Qt.Key_Right:
            self._timeline.translate_timeline_right()
        elif key == Qt.Key_Up or key == Qt.Key_PageUp:
            self._handle_zoom_in_clicked()
        elif key == Qt.Key_Down or key == Qt.Key_PageDown:
            self._handle_zoom_out_clicked()

    def handle_destroy(self, args):
        self._timeline.handle_close()

    def handle_close(self, event):
        self.shutdown_all()

        event.accept()

    def _resizeEvent(self, event):
        # TODO The -2 allows a buffer zone to make sure the scroll bars do not
        # appear when not needed. # On some systems (Lucid) this doesn't function
        # properly # need to look at a method to determine the maximum size of the
        # scene that # will maintain a proper no scrollbar fit in the view.
        self.graphics_view.scene().setSceneRect(
            0, 0, self.graphics_view.width() - 2,
            max(self.graphics_view.height() - 2, self._timeline._timeline_frame._history_bottom))

    def _handle_publish_clicked(self, checked):
        self._timeline.set_publishing_state(checked)

    def _handle_play_clicked(self, checked):
        if checked:
            self.play_button.setIcon(self.pause_icon)
            self._timeline.navigate_play()
        else:
            self.play_button.setIcon(self.play_icon)
            self._timeline.navigate_stop()

    def _handle_next_clicked(self):
        self._timeline.navigate_next()
        self.play_button.setChecked(False)
        self.play_button.setIcon(self.play_icon)

    def _handle_previous_clicked(self):
        self._timeline.navigate_previous()
        self.play_button.setChecked(False)
        self.play_button.setIcon(self.play_icon)

    def _handle_faster_clicked(self):
        self._timeline.navigate_fastforward()
        self.play_button.setChecked(True)
        self.play_button.setIcon(self.pause_icon)

    def _handle_slower_clicked(self):
        self._timeline.navigate_rewind()
        self.play_button.setChecked(True)
        self.play_button.setIcon(self.pause_icon)

    def _handle_begin_clicked(self):
        self._timeline.navigate_start()

    def _handle_end_clicked(self):
        self._timeline.navigate_end()

    def _handle_thumbs_clicked(self, checked):
        self._timeline._timeline_frame.toggle_renderers()

    def _handle_zoom_all_clicked(self):
        self._timeline.reset_zoom()

    def _handle_zoom_out_clicked(self):
        self._timeline.zoom_out()

    def _handle_zoom_in_clicked(self):
        self._timeline.zoom_in()

    def _handle_topicsList_clicked(self, qModelIndex):
        row = qModelIndex.row()
        if self.topicsListViewModel.item(row).checkState() == Qt.Unchecked:
            self.topicsListViewModel.item(row).setCheckState(Qt.Checked)
        elif self.topicsListViewModel.item(row).checkState() == Qt.Checked:
            self.topicsListViewModel.item(row).setCheckState(Qt.Unchecked)

    def _handle_select_all(self):
        for i in range(self.topicsListViewModel.rowCount()):
            item = self.topicsListViewModel.item(i)
            item.setCheckState(Qt.Checked)

    def _handle_unselect_all(self):
        for i in range(self.topicsListViewModel.rowCount()):
            item = self.topicsListViewModel.item(i)
            item.setCheckState(Qt.Unchecked)

    def _handle_load_clicked(self):
        if os.path.exists(self.lineEdit_bag_file_path.text()):
            self.last_open_dir = self.lineEdit_bag_file_path.text()
        filename,  _ = QFileDialog.getOpenFileName(self, self.tr('Load from Files'), self.last_open_dir, self.tr('Bag files {.bag} (*.bag)'))
        last_topics_selection = self._get_selected_topics()
        if filename and os.path.exists(filename):
            self.topicsListViewModel.clear()
            self.last_open_dir = QFileInfo(filename).absoluteDir().absolutePath()
            self.lineEdit_bag_file_path.setText(self.last_open_dir)
            self.bagfile_name = os.path.basename(filename)[0:-4]
            self.load_bag(filename)
            # After loading bag(s), force a resize event on the bag widget so that
            # it can take the new height of the timeline into account (and show
            # the scroll bar if necessary)
            self._timeline._timeline_frame._layout()
            for topic in self._timeline._get_topics():
                item = QStandardItem(topic)
                item.setEditable(False)
                item.setCheckable(False)
                check = Qt.Checked if topic in last_topics_selection else Qt.Unchecked
                item.setCheckState(check)
                self.topicsListViewModel.appendRow(item)
            self.topicsListView.setModel(self.topicsListViewModel)
            self._resizeEvent(QResizeEvent(self.size(), self.size()))

    def load_buttons_status(self, status):
        self.pushButton_load.setEnabled(status)
        self.play_button.setEnabled(status)
        self.thumbs_button.setEnabled(status)
        self.zoom_in_button.setEnabled(status)
        self.zoom_out_button.setEnabled(status)
        self.zoom_all_button.setEnabled(status)
        self.next_button.setEnabled(status)
        self.previous_button.setEnabled(status)
        self.faster_button.setEnabled(status)
        self.slower_button.setEnabled(status)
        self.begin_button.setEnabled(status)
        self.end_button.setEnabled(status)
        self.pushButton_export.setEnabled(status)

    def load_bag(self, filename):
        qDebug("Loading '%s'..." % filename.encode(errors='replace'))
        # QProgressBar can EITHER: show text or show a bouncing loading bar,
        #  but apparently the text is hidden when the bounding loading bar is
        #  shown
        # self.progress_bar.setRange(0, 0)
        self.set_status_text.emit("Loading '%s'..." % filename)
        # progress_format = self.progress_bar.format()
        # progress_text_visible = self.progress_bar.isTextVisible()
        # self.progress_bar.setFormat("Loading %s" % filename)
        # self.progress_bar.setTextVisible(True)

        try:
            bag = rosbag.Bag(filename)
            self.load_buttons_status(False)
            self._timeline.add_bag(bag)
            self.load_buttons_status(True)
            # put the progress bar back the way it was
            qDebug("Done loading '%s'" % filename.encode(errors='replace'))
            self.set_status_text.emit("Bag file has been loaded.")
        except rosbag.ROSBagException as e:
            print("Loading '%s' failed due to: %s" % (filename.encode(errors='replace'), e))
            self.set_status_text.emit("Loading '%s' failed due to: %s" % (filename, e))

        # self.progress_bar.setFormat(progress_format)
        # self.progress_bar.setTextVisible(progress_text_visible) # causes a segfault :(
        # self.progress_bar.setRange(0, 100)
        # self clear loading filename
    
    def _get_selected_topics(self):
        topics = self._timeline._get_topics()
        topics_selection = [
            topics[i] 
            for i in range(self.topicsListViewModel.rowCount()) 
            if self.topicsListViewModel.item(i).checkState() == Qt.Checked
        ]
        return topics_selection

    def _handle_save_clicked(self):
        if os.path.exists(self.lineEdit_export_path.text()):
            self.last_open_saving_dir = self.lineEdit_export_path.text()
        topics_selection = self._get_selected_topics()
        if topics_selection is not None and topics_selection != [] and None not in topics_selection:
            start_stamp = None
            end_stamp = None
            path_to_save = QFileDialog.getExistingDirectory(self, self.tr('Choose dir to save data'), self.last_open_saving_dir)
            if path_to_save and os.path.exists(path_to_save):
                self.last_open_saving_dir = path_to_save
                self.lineEdit_export_path.setText(self.last_open_saving_dir)
                path_to_save = os.path.join(path_to_save, self.bagfile_name)
                try:
                    if not os.path.exists(path_to_save):
                        os.mkdir(path_to_save)
                    self._timeline.extract_data_from_bag(topics_selection, path_to_save, start_stamp, end_stamp)
                except Exception as e:
                    QMessageBox(QMessageBox.Warning, 'rqt_bag', 'Error create the folder {} for exporting: {}'.format(path_to_save, str(e)), QMessageBox.Ok).exec_()
        else:
            QMessageBox(QMessageBox.Warning, 'rqt_bag', 'Please select 1 topic at least.', QMessageBox.Ok).exec_()

    def _set_status_text(self, text):
        self.progress_bar.setTextVisible(False)
        if text:
            self.progress_bar.setFormat(text)
            self.progress_bar.setTextVisible(True)

    def _update_status_bar(self):
        if self._timeline._timeline_frame.playhead is None or self._timeline._timeline_frame.start_stamp is None:
            return
        # TODO Figure out why this function is causing a "RuntimeError: wrapped
        # C/C++ object of %S has been deleted" on close if the playhead is moving
        try:
            # Background Process Status
            self.progress_bar.setValue(self._timeline.background_progress)

            # Raw timestamp
            self.stamp_label.setText('%.9fs' % self._timeline._timeline_frame.playhead.to_sec())

            # Human-readable time
            self.date_label.setText(
                bag_helper.stamp_to_str(self._timeline._timeline_frame.playhead))

            # Elapsed time (in seconds)
            self.seconds_label.setText(
                '%.3fs' % (
                    self._timeline._timeline_frame.playhead - self._timeline._timeline_frame.start_stamp).to_sec())

            # File size
            self.filesize_label.setText(bag_helper.filesize_to_str(self._timeline.file_size()))

            # Play speed
            spd = self._timeline.play_speed
            if spd != 0.0:
                if spd > 1.0:
                    spd_str = '>> %.0fx' % spd
                elif spd == 1.0:
                    spd_str = '>'
                elif spd > 0.0:
                    spd_str = '> 1/%.0fx' % (1.0 / spd)
                elif spd > -1.0:
                    spd_str = '< 1/%.0fx' % (1.0 / -spd)
                elif spd == 1.0:
                    spd_str = '<'
                else:
                    spd_str = '<< %.0fx' % -spd
                self.playspeed_label.setText(spd_str)
            else:
                self.playspeed_label.setText('')
        except:
            return
    # Shutdown all members

    def shutdown_all(self):
        self._timeline.handle_close()
