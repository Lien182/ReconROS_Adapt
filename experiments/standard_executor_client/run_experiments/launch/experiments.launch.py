from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    inverse_client = Node(
        package="reference_application_inverse_client",
        node_executable="inverse_client",
        output='screen'
    )
    mnist_client = Node(
        package="reference_application_mnist_client",
        node_executable="mnist_client",
        output='screen'
    )
    sobel_client = Node(
        package="reference_application_sobel_client",
        node_executable="sobel_client",
        output='screen'
    )
    sort_client = Node(
        package="reference_application_sort_client",
        node_executable="sort_client",
        output='screen'
    )
    periodic_client = Node(
        package="reference_application_periodic_client",
        node_executable="periodic_client",
        output='screen'
    )
    ld.add_action(inverse_client)
    ld.add_action(mnist_client)
    ld.add_action(sobel_client)
    ld.add_action(sort_client)
    ld.add_action(periodic_client)
    return ld
