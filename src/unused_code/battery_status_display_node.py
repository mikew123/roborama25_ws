# created with copilot on vscode
# This script subscribes to a battery status topic and displays the voltage on an LCD screen.
# It uses the rclpy library for ROS 2 and assumes you have an LCD library for your specific hardware.
# Make sure to replace 'some_lcd_library' with the actual library you are using for your LCD screen.
# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
#from some_lcd_library import LCD  # Replace with the actual LCD library you are using
import pygame

class BatteryStatusDisplayNode(Node):
    def __init__(self):
        super().__init__('battery_status_display_node')
        # self.lcd = LCD()  # Initialize the LCD screen
        # self.lcd.clear()
        # self.lcd.write("Starting...")
        pygame.init()
        self.screen = pygame.display.set_mode((800, 480))  # Set resolution
        pygame.display.set_caption("Battery Status")
        self.font = pygame.font.Font(None, 74)  # Set font and size
        self.screen.fill((0, 0, 0))  # Black background
        pygame.display.update()
        
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_status_callback,
            10
        )

    def battery_status_callback(self, msg: BatteryState):
        voltage = msg.voltage
        # self.lcd.clear()
        # self.lcd.write(f"Voltage: {voltage:.2f}V")
        self.screen.fill((0, 0, 0))  # Clear screen with black background
        text = self.font.render(f"Voltage: {voltage:.2f}V", True, (255, 255, 255))  # White text
        self.screen.blit(text, (50, 200))  # Display text at position
        pygame.display.update()
        self.get_logger().info(f"Displayed Voltage: {voltage:.2f}V")

    def cleanup(self):
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatusDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.lcd.clear()
        # node.lcd.write("Shutting down...")
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()