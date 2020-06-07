// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#ifndef __RS_CONVERTER_CONVERTER_PLY_H
#define __RS_CONVERTER_CONVERTER_PLY_H


#include "../converter.hpp"


namespace rs2 {
    namespace tools {
        namespace converter {

            inline std::string pretty_time(std::chrono::nanoseconds d)
            {
                auto hhh = std::chrono::duration_cast<std::chrono::hours>(d);
                d -= hhh;
                auto mm = std::chrono::duration_cast<std::chrono::minutes>(d);
                d -= mm;
                auto ss = std::chrono::duration_cast<std::chrono::seconds>(d);
                d -= ss;
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(d);

                std::ostringstream stream;
                stream << std::setfill('0') << std::setw(3) << hhh.count() << ':' <<
                       std::setfill('0') << std::setw(2) << mm.count() << ':' <<
                       std::setfill('0') << std::setw(2) << ss.count() << '.' <<
                       std::setfill('0') << std::setw(3) << ms.count();
                return stream.str();
            }

            class converter_ply : public converter_base {
            protected:
                std::string _filePath;

            public:
                converter_ply(const std::string& filePath)
                    : _filePath(filePath)
                {
                }

                std::string name() const override
                {
                    return "PLY converter";
                }

                void convert(rs2::frameset& frameset) override
                {
                    static int counter;

                    rs2::pointcloud pc;
                    start_worker(
                        [this, &frameset, pc]() mutable {
                            auto frameDepth = frameset.get_depth_frame();
                            auto frameColor = frameset.get_color_frame();

                            if (frameDepth && frameColor) {
                                if (frames_map_get_and_set(rs2_stream::RS2_STREAM_ANY, frameDepth.get_frame_number())) {
                                    return;
                                }

                                pc.map_to(frameColor);

                                auto points = pc.calculate(frameDepth);

                                double param, fractpart, intpart;
                                param = frameDepth.get_timestamp()/1000;
                                fractpart = modf (param , &intpart);
                                std::stringstream filename;
                                filename << counter
                                    << "_" << static_cast<uint32_t>(trunc(param)) << "." << static_cast<uint32_t>(round(fractpart * 1000000000))
                                    << ".ply";

                                points.export_to_ply(filename.str(), frameColor);
                                counter ++;
                            }
                        });
                }
            };

        }
    }
}


#endif
