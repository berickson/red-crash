
#pragma once

//#include <iostream> // do I need this ?
#include "depthai/depthai.hpp"


class StereoPipeline{
    public:
    StereoPipeline() = default;
    ~StereoPipeline() = default;

    void initDepthaiDev();

    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedImageStreams();

    private:
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opImageStreams;
    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;

};
