# semap

SEMAP is a *Semantic Environment Mapping Framework* that provides a database system to store large-scale geometric maps in combination with semantic knowledge in order to create a multi-modal environment model that can represent a dynamic environment over time. The framework features an object-focused approach to describing the environment, where each object combines information about its semantics, naming what kind of object it is and what features it has, and its geometric shape and position in the environment. To enable the support of dynamic environments, SEMAP supports the description of articulated objects and features a transformation tree to allow flexible repositioning of objects.

The framework is build upon PostGIS and the SFCGAL plugin for PostGIS, it further makes use of SQLAlchemy to interface the SQL layer of PostGIS. SEMAP is completely integrated into ROS and brings a large set of services to address its query system, view the environment model and modify its components.

This repository hold the database implementation of the semantic environment mapping framework SEMAP.

But there are several other semap packages, that embed SEMAP into ROS:

* [semap_msgs](https://www.github.com/hdeeken/semap_msgs)
* [semap_ros](https://www.github.com/hdeeken/semap_ros)
* [semap_env](https://www.github.com/hdeeken/semap_env)
* [rviz_semap_plugins](https://www.github.com/hdeeken/rviz_semap_plugins)

The SEMAP framework is publicly available under GPL2 license. If you are interested in SEMAP and want to give the software a try, we are glad to support you!

Please contact the current maintainer, Henning Deeken ( hdeeken at uos.de ) and see the [project's website](http://www.las-vegas.uni-osnabrueck.de/related-projects/semap/) for more information.

