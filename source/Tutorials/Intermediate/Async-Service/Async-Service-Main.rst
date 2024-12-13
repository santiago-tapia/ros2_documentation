.. redirect-from::

   Async-Service
   Tutorials/Async-Service

Writing a service with an asyncronous client node
==================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

**Goal:** Understanding asyncronism in the context of services.

**Tutorial level:** Intermediate

**Time:** 20 minutes

Background
----------

For information on how to write a basic client/server service see 
:doc:`checkout this tutorial <../../Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client>`.

**All** service clients in ROS2 are **asyncronous**, in this tutorial an example of service client is 
studied to provide some insight of that fact while explaining the reason for being asyncronous and some
consequences about timing. 

Consider the following diagram: 

.. image:: images/sync-client-diagram.png
   :target: images/sync-client-diagram.png
   :alt: A sequence diagram that show how a sync-client waits for the response

It shows an syncronous client, the client makes a request and waits for the response, that is, 
its thread is not running, it is stopped until the response is returned.

On the other hand, in the following diagram an asyncronous ROS2 client is showed:

.. In the definition diagram there is an invisible interaction, in white color, otherwise the activation bar could not be deactivated.


.. image:: images/async-client-diagram.png
   :target: images/async-client-diagram.png
   :alt: A sequence diagram that show how a async-client is not waiting but doing something else


In general, an asyncronous client is running after making the request, of course, it could
be stopped by invoking a waiting function or at any other blocking function, but this 
diagram shows a ROS2 client, and, in ROS2 in order to get the response, the thread has to
be spinning, that is: it has to be waiting for any incoming events (topics, timers...) 
including the response event. This fact is showed in the diagram: when the ``Event A`` is received,
its callback is executed (the client is *activated*) and, afterwards, the response is 
received and the client executes the callback for the response. 

Since any service client in ROS2 needs to be spinning to receive the response for the 
server. That means that **all clients in ROS2 are asyncronous** by design (they can not be
*just waiting*, they can wait while spinning).

Thus, if a node **is** already **spinning** and has requested a service
**inside** a callback (any callback), the best approach is just let the callback finish,
you can process the response later in another callback in the future. You might find that
this approach is not obvious because the code to be executed after receiving the response
is not write inmediately after making the request, but you will get use to it, it
is just to write or look for that piece of code somewhere else.

This tutorial shows how to write an asyncronous client that works like in the diagram.

Prerequisites
-------------

In beginner tutorials, you learned how to :doc:`create a workspace <../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>` 
and :doc:`create a package <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`.

In :doc:`Writing a Simple cpp Service and Client <../../Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client>` you
learned how to define a custom service that adds two ints, we will use that service again. 

The source code in this tutorial is at `rclcpp examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/services>`,
you might get the code there directly. 

Tasks
------

1 Creating the server package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In a shell run:

.. code-block:: bash

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 examples_rclcpp_delayed_service --dependencies rclcpp example_interfaces

Update ``package.xml`` as usual.

1.1 Write the service server node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Inside the ``examples_rclcpp_delayed_service/src`` directory, create a new file called ``main.cpp``
and paste the following code within:

.. code-block:: C++

   #include <memory>
   #include "example_interfaces/srv/add_two_ints.hpp"
   #include "rclcpp/rclcpp.hpp"

   class DelayedSumService : public rclcpp::Node
   {
   public:
   DelayedSumService()
   : Node("delayed_service")
   {
      // Declares a parameter for delaying (default to 2.0 seconds)
      this->declare_parameter("response_delay", 2.0);

      service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
         "add_two_ints", std::bind(
                           &DelayedSumService::add_two_ints_callback, this, std::placeholders::_1,
                           std::placeholders::_2));

      RCLCPP_INFO(this->get_logger(), "DelayedSumService is ready.");
   }

   private:
   void add_two_ints_callback(
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
   {
      // Gets parameter value
      double delay;
      this->get_parameter("response_delay", delay);

      auto result = request->a + request->b;
      RCLCPP_INFO_STREAM(
         this->get_logger(),
         "Request:" << request->a << " + " << request->b << " delayed " << delay << " seconds");

      // Simulates the delay
      std::this_thread::sleep_for(std::chrono::duration<double>(delay));

      response->sum = result;
      RCLCPP_INFO_STREAM(this->get_logger(), "Response: " << result);
   }

   rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
   };

   int main(int argc, char ** argv)
   {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<DelayedSumService>();
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
   }


Update ``CMakeLists.txt`` to build the executable: add the following
lines to it (after finding packages):


.. code-block:: console

   add_executable(service_main main.cpp)
   ament_target_dependencies(service_main rclcpp example_interfaces)

   install(TARGETS service_main DESTINATION lib/${PROJECT_NAME})


Then install dependencies if you need: 

.. code-block:: bash

   rosdep install -i --from-path src --rosdistro {DISTRO} -y


And build as usual:

.. code-block:: bash

   colcon build


1.2 Examine the server code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. Note::

   This package is NOT a real service server example, but an
   instrument to experiment and understand the consecuences of
   timing in services. It includes an artificial and **unnecessary**
   delay in responding the requests. Nevertheless, it could be
   used as an example if you remove the delay.

Actually, there is no relevant items here. Only note that the
callback that attends the request is slept for a given amount of
seconds. The rest of the node is quite standard.

2 Creating the client package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 examples_rclcpp_async_recv_cb_client --dependencies rclcpp example_interfaces


Update ``package.xml`` as usual.

2.1 Write the service client node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Inside the ``examples_rclcpp_async_recv_cb_client/src`` directory, create a new file called ``main.cpp``
and paste the following code within:

.. code-block:: C++

   #include <rclcpp/rclcpp.hpp>
   #include <example_interfaces/srv/add_two_ints.hpp>
   #include <std_msgs/msg/int32.hpp>

   class AsyncReceiveCallbackClient : public rclcpp::Node
   {
   public:
   AsyncReceiveCallbackClient()
   : Node("examples_rclcpp_async_recv_cb_client")
   {
      // Create AddTwoInts client
      client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

      // Wait until service is avaible
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
         RCLCPP_ERROR(this->get_logger(), "Service is not available, trying again after 1 second");
      }

      // Create a subcription to an input topic
      subscription_ = this->create_subscription<std_msgs::msg::Int32>(
         "input_topic", 10,
         std::bind(&AsyncReceiveCallbackClient::topic_callback, this, std::placeholders::_1));

      // Create a publisher for broadcasting the result
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("output_topic", 10);

      RCLCPP_INFO(this->get_logger(), "DelayedSumClient Initialized.");
   }

   private:
   void topic_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
   {
      RCLCPP_INFO(this->get_logger(), "Received %d at topic.", msg->data);
      if (msg->data >= 0) {
         RCLCPP_INFO(this->get_logger(), "  Input topic is %d >= 0. Requesting sum...", msg->data);

         // Create request to sum msg->data + 100
         auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
         request->a   = msg->data;
         request->b   = 100;

         // Calls the service and bind the callback to receive response (not blocking!)
         auto future_result = client_->async_send_request(
         request,
         std::bind(
            &AsyncReceiveCallbackClient::handle_service_response, this, std::placeholders::_1));
      } else {
         RCLCPP_INFO(this->get_logger(), "  Input topic is %d < 0. No request is sent", msg->data);
      }
   }

   // Callback to receive response (call inside the spinning method like any other callback)
   void handle_service_response(
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
   {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Response: %ld", response->sum);

      // Publish response at output topic
      auto result_msg = std_msgs::msg::Int32();
      result_msg.data = response->sum;
      publisher_->publish(result_msg);
   }

   rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
   rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
   };

   int main(int argc, char ** argv)
   {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<AsyncReceiveCallbackClient>());
   rclcpp::shutdown();
   return 0;
   }


Update ``CMakeLists.txt`` to build the executable: add the following
lines to it (after finding packages):

.. code-block:: console

   add_executable(client_main main.cpp)
   ament_target_dependencies(client_main rclcpp std_msgs example_interfaces)

   install(TARGETS client_main DESTINATION lib/${PROJECT_NAME})

And build as usual:

.. code-block:: bash

   colcon build

2.2 Examine the client code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The code in this node:

* Creates a service client:

   .. code-block:: C++

      client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

* Waits for the service server to be avaible at constructing the node object: 

   .. code-block:: C++

      while (!client_->wait_for_service(std::chrono::seconds(1))) {
         RCLCPP_ERROR(this->get_logger(), "Service is not available, trying again after 1 second");
      }

* And creates a suscriber and a publisher (nothing interesting here).

The node implements two callbacks, first one is for the subcription: ``topic_callback``,
the request is made here, **inside** this callback:

.. code-block:: C++

   void topic_callback(const std::shared_ptr<std_msgs::msg::Int32> msg)
   {
      RCLCPP_INFO(this->get_logger(), "Received %d at topic.", msg->data);
      if (msg->data >= 0) {
         RCLCPP_INFO(this->get_logger(), "  Input topic is %d >= 0. Requesting sum...", msg->data);

         // Create request to sum msg->data + 100
         auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
         request->a   = msg->data;
         request->b   = 100;

         // Calls the service and bind the callback to receive response (not blocking!)
         auto future_result = client_->async_send_request(
         request,
         std::bind(
            &AsyncReceiveCallbackClient::handle_service_response, this, std::placeholders::_1));
      } else {
         RCLCPP_INFO(this->get_logger(), "  Input topic is %d < 0. No request is sent", msg->data);
      }
   }

This callback check the topic value and, if greater or equals to zero, prepare a request to
the service using the new topic value and `100` as arguments, and make the request itself.

The important items about ``async_send_request`` are:

* It is called inside a callback, that is, it is executed in the thread that
  is spinning the node.

* It is not blocking, that is, it will return almost inmediately. So it will
  not block the execution of the thread.

* It provides a callback as an argument, ``AsyncReceiveCallbackClient::handle_service_response``,
  that is where the code will *jump* when the response is received.

* There is **not** any other sentence after it in ``topic_callback``, so the
  execution will leave this callback and return to the spinning method.

* Just remember, in order to receive the response the node should be spinning.

* The ``future_result`` object could be ignored because it will be received
  at ``handle_service_response``, but you might use it to track the
  *state* of the request if necessary.

The second callback is for receiving the server response. Note that,
being a callback, it will be executed at the spinning thread. The code is
quite simple:

.. code-block:: C++

  void handle_service_response(
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %ld", response->sum);

    // Publish response at output topic
    auto result_msg = std_msgs::msg::Int32();
    result_msg.data = response->sum;
    publisher_->publish(result_msg);
  }

The response is given in the parameter, `future`, it is obtained in the first
line and logged. Then, the response could be processed as required, here, just
as an example, it is published in a topic.

.. note::

   Compare to the code of an hypotethical alternative syncronous client the difference is where the code
   to be executed *after* getting the result is written. In a syncronous call it is right *after*
   the sentence calling the request, while in an asyncronous request it is at **another callback**.

Installing the examples directly
---------------------------------

You might get the packages directly from code sources (clone the git
repository in a workspace and colcon build them) or if you
are using Ubuntu and you follow the `installation instructions <https://docs.ros.org/en/{REPOS_FILE_BRANCH}/Installation/Ubuntu-Install-Binary.html>`,
you can install them using apt for your ROS 2 distro:

.. code-block:: bash

   sudo apt install ros-{REPOS_FILE_BRANCH}-examples_rclcpp_async_recv_cb_client ros-{REPOS_FILE_BRANCH}-examples_rclcpp_delayed_service

Study how client and server interact
------------------------------------

Whatever you write the package or install directly the example, this section
provides some cases of study to show how client and serve interact with each
other and the impact of time execution in that interaction.

Discover available components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To see what packages that contains `examples_` are registered and available
in your workspace, execute the following command in a terminal:

.. code-block:: bash

   ros2 pkg list | grep examples_

The terminal will show a list of packages from ros2_examples, actually,
the list of packages whose name starts with `examples_`. At least you
should get:

.. code-block:: text

   examples_rclcpp_async_recv_cb_client
   examples_rclcpp_delayed_service

Just remember to source the workspace if you don't.

Run the delayed server
^^^^^^^^^^^^^^^^^^^^^^

Start a new terminal and run:

.. code-block:: bash

   ros2 run examples_rclcpp_delayed_service service_main

The service will start, in another terminal run:

.. code-block:: bash

   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 5}"

After a short delay you will get the response, return to the terminal
where you launch the server, you will have there two INFO log messages
showing the time at the incoming request and the time when the response
was sent.

.. note::

   As already said, this server is designed NOT to be an example, but an
   emulator of a service that will take a significant amount of time to
   compute the response.

You might fine tune the timing by running:

.. code-block:: bash

   ros2 param set /delayed_service response_delay 2.5

Being 2.5 the new delay in seconds. Kept that value as the delay to have
plenty of time to run next steps.

Run the asyncronous client
^^^^^^^^^^^^^^^^^^^^^^^^^^

Start a new terminal and run (source the workspace, if you have to):

.. code-block:: bash

   ros2 ros2 run examples_rclcpp_async_recv_cb_client client_main

This node doesn't make a request on launching, instead the call for service
is done when a topic is received, that is, the call to `async_send_request`
is **inside** a ros2 callback. So you have to trigger the request by publishing
to a topic, start a third terminal and run:

.. code-block:: bash

   ros2 topic pub --once /input_topic std_msgs/msg/Int32 "data: 5"

Check the messages in both terminals, the one for the server and the one
for the client. You will find that, as you did manually before, the client made
a request and a bit later it received the response. On the server side
you will see exactly the same messages, no news there.

Now, Why is this client asyncronous? Being asyncronous means that the
program is not stopped waiting for a result, insteads it keeps running
doing other things while waiting for the response. Actually, this is
the case of **all** ROS2 service clients because they all have to keep
spinning to keep the incoming response from the rclcpp layer.

.. note::

   In this example, the client is the part that is asyncronous. Applying the
   term asyncronous to the server doesn't make any sense in this example.

Let's try to see it in action, just run these commands one after the
other (if you are not fast enough, just set the delay time to a higher
value), copy-paste them to your terminal will also work:

.. code-block:: bash

   ros2 topic pub --once /input_topic std_msgs/msg/Int32 "data: 10"
   ros2 topic pub --once /input_topic std_msgs/msg/Int32 "data: 15"

Check the client terminal, you will get something similar to:

.. code-block:: text

   [INFO] [1733332216.902893640] [examples_rclcpp_async_recv_cb_client]: Received 10 at topic.
   [INFO] [1733332216.902928394] [examples_rclcpp_async_recv_cb_client]:   Input topic is 10 >= 0. Requesting sum...
   [INFO] [1733332218.457559892] [examples_rclcpp_async_recv_cb_client]: Received 15 at topic.
   [INFO] [1733332218.457593992] [examples_rclcpp_async_recv_cb_client]:   Input topic is 15 >= 0. Requesting sum...
   [INFO] [1733332219.403816764] [examples_rclcpp_async_recv_cb_client]: Response: 110
   [INFO] [1733332221.904430291] [examples_rclcpp_async_recv_cb_client]: Response: 115

Since the client **is** asyncronous, it keeps spinning, and thus receiving
topic messages, in the previous logs the topics for 10 and 15 were received at
a time ending in 16 and 18 seconds respectively, and the responses were received
later. That is, two request were done in a row before getting the results and
later they were also received one after the other. But, why the second response
takes more that 2.5 seconds?

Check now the terminal that runs the server, you will see something similar to:

.. code-block:: text

   [INFO] [1733332216.903081355] [delayed_service]: Request:10 + 100 delayed 2.5 seconds
   [INFO] [1733332219.403276302] [delayed_service]: Response: 110
   [INFO] [1733332219.403700193] [delayed_service]: Request:15 + 100 delayed 2.5 seconds
   [INFO] [1733332221.903918827] [delayed_service]: Response: 115

The server logs a message in its service callback, the client made the second
call at a time whose seconds are 18.45, but the message here is logged at 19.40,
what happens?

Actually, it is very simple, the server is spinning, just like any other node
and this server only has one thread, so the first callback, the one with
arguments 10+100 **blocks** the spinning thread until it completes and returns.
Then the spinning takes control again, looks for another incoming request and
calls again the callback method with the new arguments: 15+100. Even if we
might think on them as parallel requests, that is not true, if a
**Single-Threaded Executor** is used, only one thread is used and, thus, the
callback are executed strictly in sequence.

The key concept here is that an asyncronous call, like in the client,
does not **block** execution, so the spinning in the client gets the control again after processing the 
callback with the request inside, that way it can execute the callback for other incoming
messages, including other topic messages **and** the incoming responses.
The client node is also run by a Single-Threaded Executor, so, callbacks are
also processed in sequence. The difference is that the callbacks in the client
return almost inmediately and so, apparently, the client is always ready.

Another lesson here is that you should make service requests with caution, if you
make requests at a high frequency you should be aware of the efficiency of your server to 
produce the responses. 

.. note::

   Actually, in any circumstance, it is a good idea to check callback execution
   times, since they **block** spinning and could produce unexpected and
   unwanted side-effects.

Just as a final note: programming a service server that takes too long in
computing the response is a potential issue in your system, this inconvenience
is a reason for using actions (among others).

Summary
--------

You have create an asyncronous client node using a design that could be
used in combination with other events in ROS2: topics, timer, etc. Its
execution model is quite simple since the node is just executed in the
default mode (single-threaded).

You have made some experiments about timing and the impact of
blocking callbacks and you should now understand better the meaning
of asyncronism and its impact in code design.
