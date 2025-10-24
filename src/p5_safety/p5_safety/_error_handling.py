from p5_interfaces.msg import Error


class ErrorHandler:
    def __init__(self, node):
        self.SEVERITIES = ["INFO", "WARNING", "ERROR", "FATAL"]

        self.info = "INFO"
        self.warning = "WARNING"
        self.error = "ERROR"
        self.fatal = "FATAL"

        self.node = node

        self.error_publisher = self.node.create_publisher(Error, 'error_message', 10)

    def report_error(self, severity, message):
        if severity in self.SEVERITIES:
            error = Error()

            error.stamp = self.node.get_clock().now().to_msg()
            error.node_name = self.node.get_name()
            error.severity = severity
            error.message = message

            self.error_publisher.publish(error)

        else:
            error = Error()

            error.stamp = self.node.get_clock().now().to_msg()
            error.node_name = self.node.get_name()
            error.severity = self.error
            error.message = f"Definition of error was not correct. Severity can only be {self.SEVERITIES} and not {severity}. Error message: {message}"

            self.error_publisher.publish(error)
