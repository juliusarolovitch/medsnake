/*
# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/


#include "hello_gui.h"
#include "ui_hello_gui.h"

ModeDisplay::ModeDisplay(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::ModeDisplay)
{
  ui->setupUi(this);

  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate


  // setup subscriber by according to the ~/chatter_topic param
  std::string listen_topic;
  chatter_sub_ = nh_->subscribe<std_msgs::String>("/medsnake_mode", 1, &ModeDisplay::chatterCallback, this);
}

ModeDisplay::~ModeDisplay()
{
  delete ui;
  delete ros_timer;
}


void ModeDisplay::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
    QApplication::quit();
}


void ModeDisplay::chatterCallback(const std_msgs::String::ConstPtr &msg){
  auto qstring_msg = QString::fromStdString( msg->data.c_str() );

  ui->chatter->setText(qstring_msg);
}
