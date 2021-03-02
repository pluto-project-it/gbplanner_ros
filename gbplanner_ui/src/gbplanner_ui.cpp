#include "gbplanner_ui.h"

namespace gbplanner_ui
{

  gbplanner_panel::gbplanner_panel(QWidget *parent) : rviz::Panel(parent)
  {
    planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
        "/planner_control_interface/std_srvs/automatic_planning");
    planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(
        "/planner_control_interface/std_srvs/stop");
    planner_client_homing = nh.serviceClient<std_srvs::Trigger>(
        "/planner_control_interface/std_srvs/homing_trigger");
    planner_client_global_planner = nh.serviceClient<planner_msgs::pci_global>(
        "pci_global");
    planner_client_load_graph = nh.serviceClient<planner_msgs::planner_string_trigger>("gbplanner/load_graph");
    planner_client_save_graph = nh.serviceClient<planner_msgs::planner_string_trigger>("gbplanner/save_graph");

    QVBoxLayout *v_box_layout = new QVBoxLayout;

    button_start_planner = new QPushButton;
    button_stop_planner = new QPushButton;
    button_homing = new QPushButton;
    button_global_planner = new QPushButton;
    button_load_graph_planner = new QPushButton;
    button_save_graph_planner = new QPushButton;

    button_start_planner->setText("Start Planer");
    button_stop_planner->setText("Stop Planer");
    button_homing->setText("Go Home");
    button_global_planner->setText("Run Global");
    button_load_graph_planner->setText("Load graph");
    button_save_graph_planner->setText("Save graph");

    v_box_layout->addWidget(button_start_planner);
    v_box_layout->addWidget(button_stop_planner);
    v_box_layout->addWidget(button_homing);

    QVBoxLayout *global_vbox_layout = new QVBoxLayout;
    QHBoxLayout *global_hbox_layout1 = new QHBoxLayout;
    QHBoxLayout *global_hbox_layout2 = new QHBoxLayout;
    QHBoxLayout *global_hbox_layout3 = new QHBoxLayout;

    QLabel *text_label_ptr = new QLabel("Frontier ID:");
    QLabel *text_label_ptr_load_graph = new QLabel("Graph path:");
    QLabel *text_label_ptr_save_graph = new QLabel("Save path:");

    global_id_line_edit = new QLineEdit();
    graph_path_load_edit = new QLineEdit();
    graph_path_save_edit = new QLineEdit();

    // Hor. box. 1
    global_hbox_layout1->addWidget(text_label_ptr);
    global_hbox_layout1->addWidget(global_id_line_edit);
    global_hbox_layout1->addWidget(button_global_planner);
    global_vbox_layout->addLayout(global_hbox_layout1);

    // Hor. box 2
    global_hbox_layout2->addWidget(text_label_ptr_load_graph);
    global_hbox_layout2->addWidget(graph_path_load_edit);
    global_hbox_layout2->addWidget(button_load_graph_planner);
    global_vbox_layout->addLayout(global_hbox_layout2);

    // Hor. box 3
    global_hbox_layout3->addWidget(text_label_ptr_save_graph);
    global_hbox_layout3->addWidget(graph_path_save_edit);
    global_hbox_layout3->addWidget(button_save_graph_planner);
    global_vbox_layout->addLayout(global_hbox_layout3);
    
    // Add vertical box.
    v_box_layout->addLayout(global_vbox_layout);

    setLayout(v_box_layout);

    connect(button_start_planner, SIGNAL(clicked()), this,
            SLOT(on_start_planner_click()));
    connect(button_stop_planner, SIGNAL(clicked()), this,
            SLOT(on_stop_planner_click()));
    connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
    connect(button_global_planner, SIGNAL(clicked()), this,
            SLOT(on_global_planner_click()));
    connect(button_load_graph_planner, SIGNAL(clicked()), this, SLOT(on_load_graph_planner_click()));
    connect(button_save_graph_planner, SIGNAL(clicked()), this, SLOT(on_save_graph_planner_click()));
  }

  void gbplanner_panel::on_save_graph_planner_click()
  {
    std::string in_string = graph_path_save_edit->text().toStdString();
    planner_msgs::planner_string_trigger srv;
    //plan_srv.request.id = id;
    srv.request.message = in_string;
    if (!planner_client_save_graph.call(srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_save_graph.getService().c_str());
    }
    else{
      ROS_INFO("Graph saved in %s", in_string.c_str());
    }
  }

  void gbplanner_panel::on_load_graph_planner_click()
  {
    std::string in_string = graph_path_load_edit->text().toStdString();
    planner_msgs::planner_string_trigger srv;
    //plan_srv.request.id = id;
    srv.request.message = in_string;
    if (!planner_client_load_graph.call(srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_load_graph.getService().c_str());
    }
  }

  void gbplanner_panel::on_start_planner_click()
  {
    std_srvs::Trigger srv;
    if (!planner_client_start_planner.call(srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                planner_client_start_planner.getService().c_str());
    }
  }

  void gbplanner_panel::on_stop_planner_click()
  {
    std_srvs::Trigger srv;
    if (!planner_client_stop_planner.call(srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                planner_client_stop_planner.getService().c_str());
    }
  }

  void gbplanner_panel::on_homing_click()
  {
    std_srvs::Trigger srv;
    if (!planner_client_homing.call(srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                planner_client_homing.getService().c_str());
    }
  }
  void gbplanner_panel::on_global_planner_click()
  {
    // retrieve ID as a string
    std::string in_string = global_id_line_edit->text().toStdString();
    // global_id_line_edit->clear();
    int id = -1;
    if (in_string.empty())
      id = 0;
    else
    {
      // try to convert to an integer
      try
      {
        id = std::stoi(in_string);
      }
      catch (const std::out_of_range &exc)
      {
        ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
        return;
      }
      catch (const std::invalid_argument &exc)
      {
        ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
        return;
      }
    }
    // check bounds on integer
    if (id < 0)
    {
      ROS_ERROR("[GBPLANNER UI] - In valid ID, must be non-negative");
      return;
    }
    // we got an ID!!!!!!!!!
    ROS_INFO("Global Planner found ID : %i", id);

    planner_msgs::pci_global plan_srv;
    plan_srv.request.id = id;
    if (!planner_client_global_planner.call(plan_srv))
    {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
                planner_client_global_planner.getService().c_str());
    }
  }
  void gbplanner_panel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }
  void gbplanner_panel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

} // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)
