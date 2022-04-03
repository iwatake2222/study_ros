import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters

class Bg_Param(Node):
    def __init__(self):
        super().__init__("cg_turtle")

    def setParam(self, red, green, blue):
        client = self.create_client(
            SetParameters,
            "/turtlesim/set_parameters".format_map(locals())
        )
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service timed out")
        req = SetParameters.Request()

        param = Parameter()
        param.name = "background_r"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = red
        req.parameters.append(param)

        param = Parameter()
        param.name = "background_g"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = green
        req.parameters.append(param)

        param = Parameter()
        param.name = "background_b"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = blue
        req.parameters.append(param)

        future = client.call_async(req)

    def getParam(self):
        client = self.create_client(
            GetParameters,
            "/turtlesim/get_parameters".format_map(locals())
        )
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service timed out")
        req = GetParameters.Request()
        req.names = ["background_r", "background_g", "background_b"]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            e = future.exception()
            raise RuntimeError(
                "Exception while calling service of node" "'{args.node_name}': {e}".format_map(locals())
            )
        print("background_r:", response.values[0].integer_value)
        print("background_g:", response.values[1].integer_value)
        print("background_b:", response.values[2].integer_value)

def main(args=None):
    rclpy.init(args=args)
    param_client = Bg_Param()
    param_client.setParam(0, 10, 100)
    param_client.getParam()

if __name__ == "__main__":
    main()
