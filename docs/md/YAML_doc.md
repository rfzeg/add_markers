## Install
After installing ROS, the yaml-cpp library gets installed as a rosdep system dependency. Rosdep gets installed (when following the steps in the official ROS wiki page) by the instruction line below:  
`$ rosdep install --from-paths -i -y src`  

## Includes
YAML 05
`#include <yaml-cpp/yaml.h>`  

To get the size of a sequence Node:  
YAML::Node.size(); 

## Build instructions 
To compile a new ROS package that uses yaml-cpp make sure the CMakeLists.txt file contains the following instruction so the compiler can link
against the yaml-cpp library:  
`target_link_libraries(${PROJECT_NAME} yaml-cpp)`  

Also the package.xml manifest has to be edited to add the following dependency:  
`<build_depend>common_rosdeps</build_depend>`  

## Troubleshooting
Error message:  
terminate called after throwing an instance of 'YAML::InvalidNode'
what():  yaml-cpp: error at line 0, column 0: invalid node; this may result from using a map iterator as a sequence iterator, or vice-versa  

Solution:  
Use the method Type() to identify if the node is map, sequence, scalar or null.

## Example 1

```cpp
void loadYAMLParams(const YAML::Node& n, const std::string& prefix)
{
	switch(n.Type())
	{
		case YAML::NodeType::Map:
		{
			for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
			{
        // load recursively
				loadYAMLParams(it->second, prefix + "/" + it->first.as<std::string>());
			}
			break;
		}
		case YAML::NodeType::Sequence:
      {
        for( YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
          {
            Config child = config.listAppendNew();
            readYamlNode( child, *it );
          }
          break;
      }
		case YAML::NodeType::Scalar:
		{
			m_params[prefix] = yamlToXmlRpc(n);

			// A dynamic parameter of the same name gets overwritten now
			m_paramJobs.erase(prefix);
			break;
		}
		default:
		{
			throw error("invalid yaml node type");
		}
	}
}
```




```cpp
void MarkerParser::readYamlNode(const YAML::Node &yaml_node)
{
  switch (yaml_node.Type())
  {
  case YAML::NodeType::Map:
  {
    for (YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it)
    {

      // option 1 (access via dereferencing)

      std::string value, key;
      key = it->first.as<std::string>();
      value = it->second.as<std::string>();

      // option 2 (access via new Yaml Node)

      // get reference to the "it" and create new YAML::Node reference out of it
      const YAML::Node& node = *it;
      std::cout << "Id: " << node["id"].as<std::string>() << "\n";
      std::cout << "hardwareId: " << node["hardwareId"].as<std::string>() << "\n\n";

      /* example file:
      sensors:
        - id: 5
          hardwareId: 28-000005a32133
          type: 1
        - id: 6
          hardwareId: 28-000005a32132
          type: 4

      */

    }
    break;
  }
  case YAML::NodeType::Sequence:
  {
    for (YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it)
    {
      // get the whole list as a vector
      std::vector<std::string> strings = it->as<std::vector<std::string>>();
      // or:   = it.as<std::vector<std::string>>();

      // or:

      // get reference to the "it" and create new YAML::Node reference out of it
      const YAML::Node &item = *it;
      // get the size of the new Node
      item.size() // YAML::Node.size()
      // get each element by accesing using positional index

      // or:

      for(YAML::const_iterator it = n.begin(); it != n.end(); ++it)
				{
					values.push_back((*it));
				}
				return values;

    }
    break;
  }
  case YAML::NodeType::Scalar:
  {
    std::string s;
    s = yaml_node.as<std::string>();
    break;
  }
  case YAML::NodeType::Null:
  {
    ROS_WARN("Found YAML Node of type NULL");
    break;
  }
  default:
  {
    ROS_WARN("Found YAML Node of unknown type (neither map, sequence, scalar nor null)");
  }
  }
}
```

## Example 3:

```cpp
bool MarkerParser::loadDataFromYaml(const std::string& filename)
{

  // Yaml File Parsing
  try
  {
    YAML::Node doc;

    MarkerParser::parseMarkersFromFile(filename, doc);

  }
  catch(YAML::ParserException& e)
  {
    ROS_ERROR("Parsing file failed: %s", e.what());
    return false;
  }
  catch(YAML::RepresentationException& e)
  {
    ROS_ERROR("Parsing file failed: %s", e.what());
    return false;
  }
  catch(std::string& e) {
    ROS_ERROR("Parsing file failed: %s",e.c_str());
    return false;
  }

  return true;
}
```

[visualization_msgs::Marker::MESH_RESOURCE](https://github.com/Minseongusg/climb_nav_source/blob/master/avoidance/avoidance/src/rviz_world_loader.cpp)