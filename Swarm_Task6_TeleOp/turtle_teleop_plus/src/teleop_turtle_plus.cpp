// teleop_turtle_plus.cpp
//
// This node controls turtlesim from the keyboard.
// Keys:
//   Arrows / W A S D  -> move
//   Space             -> temporary speed boost
//   R / G / B         -> change pen color
//   anything else     -> reset pen to black and stop

#include <chrono>       //used for timing, basically how many inputs per second
#include <termios.h>    // for turning off terminal buffering
#include <unistd.h>     // for read()
#include "rclcpp/rclcpp.hpp"    //main ros2 library
#include "geometry_msgs/msg/twist.hpp"   //control linear and angular velocity
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;

class TurtleTeleopNode : public rclcpp::Node
{
public:
  TurtleTeleopNode() : rclcpp::Node("turtle_teleop_node")
  {
    // publisher to move the turtle
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // client to change pen color
    pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    // set some basic speeds
    normal_linear_speed_  = 2.0;
    normal_angular_speed_ = 2.0;
    boost_multiplier_     = 2.5;
    is_boosting_          = false;
    current_linear_       = 0.0;
    current_angular_      = 0.0;

    // put terminal in "raw" mode so 1 key = 1 character
    enableRawMode();

    // run the main loop every 50 ms
    timer_ = this->create_wall_timer(50ms, std::bind(&TurtleTeleopNode::mainLoop, this));

    RCLCPP_INFO(this->get_logger(), "Keyboard teleop started.");
    RCLCPP_INFO(this->get_logger(), "Use arrows/WASD, SPACE=boost, R/G/B=pen color.");
  }

  ~TurtleTeleopNode()
  {
    disableRawMode();
  }

private:
  // This is called every 50 ms
  void mainLoop()
  {
    int key = readKey();   // -1 if no key pressed
    if (key != -1) {
      handleKey(key);
    }

    // turn off boost after some time
    if (is_boosting_ && this->now() > boost_end_time_) {
      is_boosting_ = false;
    }

    // publish movement
    geometry_msgs::msg::Twist msg;
    double boost = is_boosting_ ? boost_multiplier_ : 1.0;
    msg.linear.x  = current_linear_  * boost;
    msg.angular.z = current_angular_ * boost;
    cmd_pub_->publish(msg);
  }

  // figure out what the key means
  void handleKey(int key)
  {
    // arrow keys come as ESC [ A/B/C/D
    if (key == 27) {                // ESC  // THESE ARE THE ASCII VALUES FOR KEYBOARD, when 27 is seen the compiler knows that its gonna expect an arrow key
      int k1 = readKey();
      int k2 = readKey();
      if (k1 == 91) {               // '['
        if (k2 == 65) { moveForward();  return; } // Up        //ascii for A
        if (k2 == 66) { moveBackward(); return; } // Down
        if (k2 == 67) { turnRight();    return; } // Right
        if (k2 == 68) { turnLeft();     return; } // Left
      }
    }

    // normal single characters
    char c = static_cast<char>(key);   

    switch (c) {
      // movement
      case 'w': case 'W':
        moveForward();
        break;
      case 's': case 'S':
        moveBackward();
        break;
      case 'a': case 'A':
        turnLeft();
        break;
      case 'd': case 'D':
        turnRight();
        break;

      // boost
      case ' ':
        startBoost();
        break;

      // pen colors
      case 'r': case 'R':
        setPenColor(255, 0, 0);
        break;
      case 'g': case 'G':
        setPenColor(0, 255, 0);
        break;
      case 'b': case 'B':
        setPenColor(0, 0, 255);
        break;

      // anything else: reset pen and stop
      default:
        setPenColor(0, 0, 0);  // black
        stopMoving();
        break;
    }
  }


  void moveForward()
  {
    current_linear_ = normal_linear_speed_;
    current_angular_ = 0.0;
  }

  void moveBackward()
  {
    current_linear_ = -normal_linear_speed_;
    current_angular_ = 0.0;
  }

  void turnLeft()
  {
    current_linear_ = 0.0;
    current_angular_ = normal_angular_speed_;
  }

  void turnRight()
  {
    current_linear_ = 0.0;
    current_angular_ = -normal_angular_speed_;
  }

  void stopMoving()
  {
    current_linear_ = 0.0;
    current_angular_ = 0.0;
  }

  void startBoost()
  {
    is_boosting_ = true;
    // boost for 0.5 seconds
    boost_end_time_ = this->now() + rclcpp::Duration::from_seconds(0.5);
  }

  /* ==== pen helper ==== */
  void setPenColor(uint8_t r, uint8_t g, uint8_t b)
  {

    if (!pen_client_->wait_for_service(0s)) {
      RCLCPP_WARN(this->get_logger(), "set_pen service not available yet");
      return;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = 3;
    request->off = 0;

    pen_client_->async_send_request(request);
  }

 
  void enableRawMode()
  {
    tcgetattr(STDIN_FILENO, &original_termios_);
    termios raw = original_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);  // chatgpt hai im ngl this part especially
    raw.c_cc[VMIN] = 0;               // Basically lets real time input happen, meaning as soon as a key is entered, that value is input   
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  }

  void disableRawMode()
  {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);   
  }

  // return pressed key, or -1 if no key was pressed
  int readKey()
  {
    unsigned char c;
    int n = read(STDIN_FILENO, &c, 1);
    if (n == 1) return c;
    return -1;
  }

  /* ==== member variables ==== */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr        pen_client_;
  rclcpp::TimerBase::SharedPtr                             timer_;

  double normal_linear_speed_;
  double normal_angular_speed_;
  double boost_multiplier_;

  double current_linear_;
  double current_angular_;
  bool   is_boosting_;
  rclcpp::Time boost_end_time_;

  termios original_termios_;   // to restore terminal settings
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

