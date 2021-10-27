
#include "Pathfinding/Objectifier.hpp"
#include "Pathfinding/Writer.hpp"
#include "albot/MapProcessing/MapProcessing.hpp"

namespace Processor {
	void process(std::shared_ptr<MapProcessing::MapInfo> info) {
		Objectifier objectifier(info);
		objectifier.run();
		Writer writer(objectifier);
		writer.write();
	}
}