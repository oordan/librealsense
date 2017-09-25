// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#ifndef LIBREALSENSE_RS2_PIPELINE_HPP
#define LIBREALSENSE_RS2_PIPELINE_HPP

#include "rs_types.hpp"
#include "rs_frame.hpp"
#include "rs_context.hpp"


namespace rs2
{
    /**
    * The pipeline simplifies the user interaction with the device and computer vision processing modules.
    * The class abstracts the camera configuration and streaming, and the vision modules triggering and threading. 
    * It lets the application focus on the computer vision output of the modules, or the device output data.
    * The pipeline can manage computer vision modules, which are implemented as a processing blocks. 
    * The pipeline is the consumer of the processing block interface, while the application consumes the 
    * computer vision interface. 
    */
    class pipeline
    {
    public:

        /**
        * Create a pipeline for processing data from a single device.
        * The caller can provide a context created by the application, usually for playback or testing purposes.
        *
        * \param[in] ctx   The context allocated by the application. Using the platform global context by default.
        */
        pipeline(context ctx = context())
            : _ctx(ctx)
        {
            rs2_error* e = nullptr;
            _pipeline = std::shared_ptr<rs2_pipeline>(
                rs2_create_pipeline(ctx._context.get(), &e),
                rs2_delete_pipeline);
            error::handle(e);
        }


        /**
        * Retrieve the device used by the pipeline.
        * The pipeline device is selected during \c commit_config() or \c start() call.
        * The method returns a valid device only after one of those device selection calls.
        * The device class provides the application access to control camera additional settings -
        * get device information, sensor options information, options value query and set, sensor specific extensions.
        * Since the pipeline controls the device streams configuration, activation state and frames reading, calling
        * the device API functions, which execute those operations, results in unexpected behavior.
        *
        * \return rs2::device The pipeline selected device
        */
        device get_device() const
        {
            rs2_error* e = nullptr;
            std::shared_ptr<rs2_device> dev(
                rs2_pipeline_get_device(_ctx._context.get(), _pipeline.get(), &e),
                rs2_delete_device);

            error::handle(e);

            return device(dev);
        }

        /**
        * Start the pipeline main streaming loop.
        * The pipeline streaming loop captures samples from the camera, and delivers them to the attached computer vision modules
        * and processing blocks, according to each module requirements and threading model.
        * The pipeline selects and activates the device upon start, according to application enabled configuration or default configuration.
        * If the application already called \c commit_config() explicitly, the pipeline tries to activate the previously selected device.
        * The device activation may fail if another application acquired ownership of one of the selected sensors.
        * During the loop execution, the application can access the camera streams by calling \c wait_for_frames() or \c poll_for_frames().
        * The streaming loop runs until the pipeline is stopped. Starting the pipeline is possible only when it is not started,
        * after creation or after \c commit_config(). If the pipeline was started, the operation takes no effect.
        */
        void start() const
        {
            rs2_error* e = nullptr;
            rs2_start_pipeline(_pipeline.get(), &e);
            error::handle(e);
        }

        /**
        * Optionally commit the camera configuration, to select the device used by the pipeline.
        * Committing the pipeline configuration explicitly provides the application access to the pipeline selected device once this
        * call returns. Thus, the application can get access to the device and sensors additional configuration before streaming starts
        * and frames processing begins.
        * Calling this method is optional, before pipeline \c start(), as the pipeline calls it internally during start if not called
        * by the application. Calling this method after pipeline \c start() is invalid.
        * The method resolves the user enabled streams and configuration requests for the device, combines them with the requirements of
        * the attached computer vision modules and processing blocks, and searches for a suitable device connected to the platform.
        * In the absence of any requests, the pipeline selects a default configuration for the default device.
        * The selected configuration is not applied to the device until \c start() is called, so the application doesn't own the device
        * sensors when this method returns. If another application acquires a sensor before start is called, start will fail.
        * Commit configuration succeeds if the selected device configuration satisfies all the attached computer vision modules, and is 
        * supported by a connected device on the platform. The method fails if the requested configuration doesn't satisfy one of the above.
        * After commit config is called, no changes to the enabled streams and features, or the pipeline computer vision modules and 
        * processing blocks, can be made, until the pipeline is stopped.
        */
        void open() const //commit_config
        {
            rs2_error* e = nullptr;
            rs2_open_pipeline(_pipeline.get(), &e);
            error::handle(e);
        }

        /**
        * Stop the pipeline main streaming loop.
        * The pipeline stops delivering samples to the attached computer vision modules and processing blocks, stops the device streaming
        * and releases the device resources used by the pipeline. It is the application's responsibility to release any frame reference it owns.
        * The pipeline remains configured with the previously enabled streams and features, so it can be reconfigured or restarted at this state.
        * To add or remove computer vision modules the application should call \c reset() to uncofigure the pipeline.
        * Calling start after stop will use the last configuration. The method takes affect only after \c start() was called.
        */
        void stop() const
        {
            rs2_error* e = nullptr;
            rs2_stop_pipeline(_pipeline.get(), &e);
            error::handle(e);
        }

        /**
        * Reset the pipeline configuration.
        * The method clears any selected camera configuration, and removes all attached computer vision modules and processing blocks.
        * After this method returns, the pipeline is back to its initial state, and the application may add computer vision modules or 
        * processing blocks, set camera configuration and call pipeline start again after this call.
        * Resetting the pipeline configuration while streaming is invalid. The application must call \c stop() before calling this method.
        */
        void reset_config(){}

        /**
        * Optionally enable a device stream explicitly, with selected stream parameters.
        * The method allows the application to request a stream with specific configuration, replacing the pipeline default selection.         
        * The method takes effect when called before the configuration is commited, through \c commit_config() or \c start(), or after 
        * \c reset_config() is called. When the pipeline has a selected device and configuration, calling this method is invalid (ignored / fails???). 
        * If no stream is explicitly enabled, the pipeline configures the device and its streams according to the attached computer vision modules
        * and processing blocks requirements, or default configuration.
        * The application can configure any of the input stream parameters according to its requirement, or set to 0 for don't care value. 
        * The pipeline accumulates the application calls for enable configuration methods, until the configuration is committed. Multiple enable stream calls
        * for the same stream with conflicting parameters (????????) override each other, and the last call is maintained.
        * Upon configuration commit, the pipeline checks for conflicts between the application configuration requests and the attached computer vision modules 
        * and processing blocks requirements, and fails if conflicts are found.       
        * 
        * \param[in] stream    Stream type to be enabled
        * \param[in] index     Stream index, used for multiple streams of the same type. 0 selects the default.
        * \param[in] width     Stream image width - for images streams
        * \param[in] height    Stream image height - for images streams
        * \param[in] format    Stream data format - pixel format for images streams, of data type for other streams
        * \param[in] framerate Stream frames per second
        */
        void enable_stream(rs2_stream stream, int index, int width, int height, rs2_format format, int framerate) const
        {
            rs2_error* e = nullptr;
            rs2_enable_pipeline_stream(_pipeline.get(), stream, index, width, height, format, framerate, &e);
            error::handle(e);
        }
        /**
        * Optionally select an explicit device by its serial number, to be used by the pipeline.
        * The conditions and behavior of this method are similar to those of \c reset_config().
        * 
        * \param[in] Serial device serial number, as returned by RS2_CAMERA_INFO_SERIAL_NUMBER
        */
        void enable_device(std::string serial) const
        {
            rs2_error* e = nullptr;
            rs2_enable_pipeline_device(_pipeline.get(), serial.c_str(), &e);
            error::handle(e);
        }
        /**
        *  remove a configuration from the pipeline
        * \param[in] stream    stream type
        */
        void disable_stream(rs2_stream stream) const
        {
            rs2_error* e = nullptr;
            rs2_disable_stream_pipeline(_pipeline.get(), stream, &e);
            error::handle(e);
        }

        /**
        *  remove all streams from the pipeline
        * \param[in] stream    stream type
        */
        void disable_all() const
        {
            rs2_error* e = nullptr;
            rs2_disable_all_streams_pipeline(_pipeline.get(), &e);
            error::handle(e);
        }

        /**
        * Wait until a new set of frames becomes available.
        * The frames set includes time-synchronized frames of each enabled stream in the pipeline. 
        * The method blocks the calling thread, and fetches the latest unread frames set. 
        * Device frames, which were produced while the function wasn't called, are dropped. To avoid frame drops, this method should be called as fast as the device frame rate. 
        * The application can maintain the frames handles to defer processing. However, if the application maintains too long history, the device may lack memory resources 
        * to produce new frames, and the following call to this method shall fail to retrieve new frames, until resources are retained. 
        * 
        * \param[in] timeout_ms   Max time in milliseconds to wait until an exception will be thrown
        * \return                 Set of time synchronized frames, one from each active stream
        */
        frameset wait_for_frames(unsigned int timeout_ms = 5000) const
        {
            rs2_error* e = nullptr;
            frame f(rs2_pipeline_wait_for_frames(_pipeline.get(), timeout_ms, &e));
            error::handle(e);

            return frameset(f);
        }       

        /**
        * Poll if a new set of frames is available and retrieve the latest set.
        * The frames set includes time-synchronized frames of each enabled stream in the pipeline.
        * The method returns without blocking the calling thread, with status of new frames available or not. If available, it fetches the latest frames set. 
        * Device frames, which were produced while the function wasn't called, are dropped. To avoid frame drops, this method should be called as fast as the device frame rate.
        * The application can maintain the frames handles to defer processing. However, if the application maintains too long history, the device may lack memory resources
        * to produce new frames, and the following calls to this method shall return no new frames, until resources are retained. 
        * 
        * \param[out] f     Frames set handle
        * \return           True if new set of time synchronized frames was stored to f, false if no new frames set is available
        */
        bool poll_for_frames(frameset* f) const
        {
            rs2_error* e = nullptr;
            rs2_frame* frame_ref = nullptr;
            auto res = rs2_pipeline_poll_for_frames(_pipeline.get(), &frame_ref, &e);
            error::handle(e);

            if (res) *f = frameset(frame(frame_ref));
            return res > 0;
        }

        /**
        * Return the selected streams profiles, used by the pipeline.
        * The pipeline streams profiles are selected during \c commit_config() or \c start() call.
        * The method returns a valid result only after one of those calls.
        *
        * \return       Vector of stream profiles
        */
        std::vector<stream_profile> get_active_streams() const
        {
            std::vector<stream_profile> results;

            rs2_error* e = nullptr;
            std::shared_ptr<rs2_stream_profile_list> list(
                rs2_pipeline_get_active_streams(_pipeline.get(), &e),
                rs2_delete_stream_profiles_list);
            error::handle(e);

            auto size = rs2_get_stream_profiles_count(list.get(), &e);
            error::handle(e);

            for (auto i = 0; i < size; i++)
            {
                stream_profile profile(rs2_get_stream_profile(list.get(), i, &e));
                error::handle(e);
                results.push_back(profile);
            }

            return results;
        }

        /**
        * Return the selected stream profile for a specific stream, used by the pipeline.
        * The pipeline streams profiles are selected during \c commit_config() or \c start() call.
        * The method returns a valid result only after one of those calls.
        *
        * \param[in] stream     The specific stream, for which the profile is requested
        * \param[in] index      The specific stream index, used for multiple streams of the same type. 0 selects the default.
        * \return               Vector of stream profiles
        */
        stream_profile get_active_streams(const rs2_stream stream, const int index = 0) const
        {
            rs2_error* e = nullptr;
            std::shared_ptr<rs2_stream_profile_list> list(
                rs2_pipeline_get_active_streams(_pipeline.get(), &e),
                rs2_delete_stream_profiles_list);
            error::handle(e);

            stream_profile profile(rs2_pipeline_get_stream_type_selection(list.get(), stream, index, &e));
            error::handle(e);

            return profile;
        }

    private:
        context _ctx;
        device _dev;
        std::shared_ptr<rs2_pipeline> _pipeline;
    };
}
#endif // LIBREALSENSE_RS2_PROCESSING_HPP
