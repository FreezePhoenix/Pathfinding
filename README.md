# Pathfinding

This is a library intended for use with ALBot-C++. It does not have to be used with that, but it is possible to build it separately. If you do so, you are responsible for setting up the build.

The way it works is that it exports a single symbol: `init`

This accepts a single argument, a pointer to an object of the following shape:

```c++
template<typename ARGUMENTS, typename RETURN = void>
class ServiceInfo {
	public:
		typedef RETURN (*HANDLER)(ARGUMENTS*);
		HANDLER child_handler = nullptr;
		GameData *G;
};
```

Where `ARGUMENTS` is `PathfindArguments`, and `RETURN` is `void*`

It will return a pointer to a method of type HANDLER: `void*(*)(PathfindArguments*)`.

PathfindArguments is of the following shape:

```c++
struct PathfindArguments {
	struct Point {
		double x;
		double y;
	};
	Point start;
	Point end;
	std::string start_map;
	std::string end_map;
};
```

Despite returning a void*, it is actually a pointer to a PathResult:

```c++
struct PathResult {
	enum Status {
		FAIL = 		0,
		SUCCESS =	1,
	} state;
	std::vector<PathfindArguments::Point> path;
};
```
