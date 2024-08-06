/****************************************************************/
/*                                                              */
/*                       Advanced Navigation                    */
/*               			NTRIP Client			  		    */
/*          Copyright 2023, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "adnav_ntrip.h"

namespace adnav::ntrip {

    bool Client::run(void) {
        // Don't restart service if it is running.
        if(service_is_running_.load()) return true;

        // Closes any current connection and opens new connection
        if (!establish_new_connection()) return false;

        // check to see if the gga string needs regeneration.
        if (!gga_update_.load()) {
            adnav::utils::GenerateGGAString(gga_string_, latitude_, longitude_,
                altitude_, gnss_fix_, sats_, hdop_);
            gga_update_.store(true);
        }

        // Form the corrections request
        std::stringstream request;

        request << "GET /" << mountpoint_ << " HTTP/1.1\r\n" <<
        "Host: " << server_ip_ << ":" << std::to_string(server_port_) << "\r\n" <<
        "Ntrip-Version: Ntrip/2.0\r\n" <<
        "User-Agent: " << user_agent_ << "\r\n" <<
        "Accept: */*\r\n" <<
        "Authorization: Basic " << encode_credentials() << "\r\n" <<
        "Ntrip-GGA: " << gga_string_ << "\r\n" <<
        "Connection: close\r\n" <<
        "\r\n";

        if (send(socket_fd_, request.str().c_str(), request.str().size(), 0) < 0) {
            log_err_fn_("Sending HTTP request failed!\r\n");
            service_failure_.store(NTRIP_SEND_REQUEST_FAILURE);
            stop();
            return false;
        }

        int ret = -1;
        std::unique_ptr<char[]> buffer = std::make_unique<char[]>(NTRIP_BUFFER_SIZE);

        int timeout = NTRIP_TIMEOUT_PERIOD * 10;
        while(timeout--) {
            // Check the socket for data
            ret = recv(socket_fd_, buffer.get(), NTRIP_BUFFER_SIZE, 0);

            // If data is present
            if (ret > 0) {
                // Reset the timeout.
                timeout = NTRIP_TIMEOUT_PERIOD * 10;

                // Store the buffered data into a string
                std::string result(buffer.get(), ret);

                // Check to see if there is a recognized header
                if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
                    (result.find("ICY 200 OK") != std::string::npos) ||
                    (result.find("HTTP/1.0 200 OK") != std::string::npos) ||
                    (result.find("SOURCETABLE 200 OK") != std::string::npos)) {

                    // Break the waiting loop.
                    break;

                } else if(result.find("HTTP/1.1 401") != std::string::npos)
                {
                    log_err_fn_("NTRIP Server Access Unauthorized.\r\n" + result + "\r\n");
                    service_failure_.store(NTRIP_UNAUTHORIZED);
                    stop();
                    return false;
                } else if(result.find("HTTP/1.1 403") != std::string::npos)
                {
                    log_err_fn_("NTRIP Server Access Forbidden.\r\n" + result + "\r\n");
                    service_failure_.store(NTRIP_FORBIDDEN);
                    stop();
                    return false;
                } else if(result.find("HTTP/1.1 404") != std::string::npos)
                {
                    log_err_fn_("NTRIP Server Resource Not Found.\r\n" + result + "\r\n");
                    service_failure_.store(NTRIP_NOT_FOUND);
                    stop();
                    return false;
                }
                else
                {
                    log_err_fn_("Unrecognized Return Header\r\n" + result + "\r\n");
                    service_failure_.store(NTRIP_UNRECOGNIZED_RETURN);
                    stop();
                    return false;
                }
            } else if (ret == 0) {
                log_err_fn_("Ntrip caster terminated connection.\r\n");
                service_failure_.store(NTRIP_REMOTE_CLOSE);
                stop();
                return false;
            }

            // Sleep for 100ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Test to see if the above loop broke due to timeout.
        if (timeout <= 0) {
            log_err_fn_("NtripCaster[" + server_ip_ + ":" + std::to_string(server_port_) +
                    " " + user_ + " " + passwd_ + " " + mountpoint_ + "] access timeout!\r\n");
            service_failure_.store(NTRIP_CONNECTION_TIMEOUT_FAILURE);
            stop();
            return false;
        }

        // Start the thread. join it if it is currently running.
        if (thread_.joinable()) thread_.join();
        thread_ = std::move(std::thread(&adnav::ntrip::Client::thread_handler, this));

        return true;
    }

    bool Client::retrieve_sourcetable(void) {
        // Closes any current on
        if (!establish_new_connection()) return false;
        std::stringstream request;
        int ret = -1;
        std::unique_ptr<char[]>  buffer = std::make_unique<char[]>(NTRIP_BUFFER_SIZE);

        // Form the Sourcetable request
        request << "GET / HTTP/1.1\r\n" <<
        "Host: " << server_ip_ << ":" << std::to_string(server_port_) << "\r\n" <<
        "Authorisation: Basic " << encode_credentials() << "\r\n" <<
        "Ntrip-Version: Ntrip/2.0\r\n" <<
        "User-Agent: " << user_agent_ << "\r\n" <<
        "Accept: */*\r\n" <<
        "Connection: close\r\n\r\n";

        if (send(socket_fd_, request.str().c_str(), request.str().size(), 0) < 0) {
            log_err_fn_("Sending SOURCETABLE request failed!\r\n");
            service_failure_.store(NTRIP_SEND_REQUEST_FAILURE);
            stop();
            return false;
        }

        // Loop for NTRIP_TIMEOUT_PERIOD in seconds.
        int timeout = NTRIP_TIMEOUT_PERIOD * 10; // converts s to ms intervals
        bool header = false;
        while(timeout--) {
            // Check the socket for data
            ret = recv(socket_fd_, buffer.get(), NTRIP_BUFFER_SIZE, 0);

            // If data is present
            if (ret > 0) {
                // reset the timeout.
                timeout = NTRIP_TIMEOUT_PERIOD * 10;

                // store the buffered data into a string
                std::string result(buffer.get(), ret);

                // Check to see if there is an recognized header or seen a recognized header previously.
                if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
                    (result.find("ICY 200 OK") != std::string::npos) ||
                    (result.find("HTTP/1.0 200 OK") != std::string::npos) ||
                    (result.find("SOURCETABLE 200 OK") != std::string::npos)) {

                    // Print out the request result using the logging function.
                    log_fn_("Sourcetable response from " + server_ip_ + ":" +
                        std::to_string(server_port_) + "\r\n" + result);
                    header = true;

                }
                else if (header)
                {
                    log_fn_(result);
                }
                else
                {
                    log_err_fn_("Unrecognized Sourcetable Header\r\n" + result + "\r\n");
                    service_failure_.store(NTRIP_UNRECOGNIZED_RETURN);
                    stop();
                    return false;
                }
            } else if (ret == 0) {
                log_fn_("Ntrip caster ended connection.\r\n");
                stop();
                return true;
            }

            // sleep for 100ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        service_failure_.store(NTRIP_CONNECTION_TIMEOUT_FAILURE);
        stop();

        return false;
    }

    void Client::stop(void) {
        // Store false in the atomic bool
        service_is_running_.store(false);

        // Close open sockets.
        #if defined(WIN32) || defined (_WIN32)
            if(socket_fd_ != INVALID_SOCKET) {
                closesocket(socket_fd_);
                WSACleanup();
                socket_fd_ = INVALID_SOCKET;
            }
        #else
            if (socket_fd_ > 0) {
                close(socket_fd_);
                socket_fd_ = -1;
            }
        #endif

        // Join the thread
        if(thread_.joinable()) thread_.join();
    }

    void Client::thread_handler(void) {
        service_is_running_.store(true);
        int ret;

        std::unique_ptr<char[]>buffer = std::make_unique<char[]>(NTRIP_BUFFER_SIZE);

        std::chrono::_V2::steady_clock::time_point gga_beg = std::chrono::steady_clock::now();
        std::chrono::_V2::steady_clock::time_point gga_end = gga_beg;
        const int gga_interval_ms = report_interval_ * 1000;

        std::chrono::_V2::steady_clock::time_point to_beg = gga_end;
        std::chrono::_V2::steady_clock::time_point to_end = to_beg;
        const int sock_timeout_ms = NTRIP_TIMEOUT_PERIOD * 1000;

        log_fn_("NtripClient service running...\r\n");

        // Loop continuously until told otherwise or an error occurs.
        while (service_is_running_.load()) {
            ret = recv(socket_fd_, buffer.get(), NTRIP_BUFFER_SIZE, 0);

            // If the socket has been closed
            if (ret == 0) {
                log_err_fn_("Remote has closed the socket!\r\n");
                service_failure_.store(NTRIP_REMOTE_CLOSE);
                break;
            } else if (ret < 0) { // socket has errored.
                // Check errno to see if it has errored due to an error
                // and not due to no data on non-blocking socket.
                if ((errno !=0) && (errno != EAGAIN) &&
                    (errno != EWOULDBLOCK) && (errno != EINTR)) {
                    log_err_fn_("Remote socket error, errno = " + std::to_string(errno) +
                        "\r\nstderr msg: " + std::strerror(errno) + "\r\n");
                    service_failure_.store(NTRIP_REMOTE_SOCKET_FAILURE);
                    break;
                }

                // no data recieved check for timeout.
                to_end = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    to_end - to_beg).count() >= sock_timeout_ms) {
                        service_failure_.store(NTRIP_CONNECTION_TIMEOUT_FAILURE);
                        break;
                }

            } else { // ret > 0 (data has been received)
                to_beg = std::chrono::steady_clock::now();
                callback_(buffer.get(), ret);
                if (ret == NTRIP_BUFFER_SIZE) continue; // recieve again.
            }

            // Get the current time.
            gga_end = std::chrono::steady_clock::now();

            // Calculate and test the gga interval.
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                gga_end - gga_beg).count() >= gga_interval_ms) {
                // reset the interval to 0
                gga_beg = std::chrono::steady_clock::now();

                // Check to see if the GGA string requires updating
                if (!gga_update_.load()) {
                    adnav::utils::GenerateGGAString(gga_string_, latitude_,
                        longitude_, altitude_, gnss_fix_, sats_, hdop_);
                    gga_update_.store(true);
                }

                // Send the GGA string
                send(socket_fd_, gga_string_.c_str(), gga_string_.size(), 0);
            }
        }
        log_fn_("NtripClient service done.\r\n");
        service_is_running_.store(false);
    }


    /**
     * @brief Function to take the current server connection loaded
     * into the class and establish a new connection to it. Calling
     * will cause any current connection to be closed before the new
     * connection is opened.
     *
     * @return success boolean.
    */
    bool Client::establish_new_connection(void) {
        stop();

        // Establish a connection with the NTRIPCaster.
        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);

        #if defined(WIN32) || defined(_WIN32)
            // startup WSA
            WSADATA ws_data;
            if(WSAStartup(MAKEWORD(2, 2), &ws_data) !=0) {
                service_failure_.store(NTRIP_CREATE_SOCK_FAILURE);
                return false;
            }
            socket_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (socket_fd_ == INVALID_SOCKET) {
                log_err_fn_("Winsock Socket Failed!\r\n");
                service_failure_.store(NTRIP_CREATE_SOCK_FAILURE);
                WSACleanup();
                return false;
            }
            server_addr.sin_addr.S_un.S_addr = inet_addr(server_ip_.c_str());
        #else
            socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (socket_fd_ == -1) {
                log_err_fn_("UNIX Socket Creation Failed!\r\n");
                service_failure_.store(NTRIP_CREATE_SOCK_FAILURE);
                return false;
            }
        #endif // defined(WIN32) || defined(_WIN32)

        // Establish new connection to server.
        if (connect(socket_fd_, reinterpret_cast<struct sockaddr *>(&server_addr),
            sizeof(server_addr)) < 0) {
                log_err_fn_("Connection to NTRIP Caster failed: " + server_ip_ + ":" +
                    std::to_string(server_port_) + " | errno = -" + std::to_string(errno) + " \r\n");
                service_failure_.store(NTRIP_CASTER_CONNECTION_FAILURE);

            #if defined(WIN32) || defined(_WIN32)
                closesocket(socket_fd_);
                WSACleanup();
            #else
                close(socket_fd_);
            #endif // defined(WIN32) || defined(_WIN32)
            return false;
        }

        // Set the sockets to be non-blocking
        #if defined(WIN32) || defined(_WIN32)
            unsigned long ul = 1;

            if(ioctlsocket(socket_fd_, FIONBIO, &ul) == SOCKET_ERROR) {
                closesocket(socket_fd);
                WSACleanup();
                return false;
            }
        #else
            int flags = fcntl(socket_fd_, F_GETFL);
            fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        #endif // defined(WIN32) || defined(_WIN32)
        return true;
    }

    /**
     * @brief Function to encode the Username and Password using Base
     * 64 Encoding for transmission over HTTP requests.
     *
     * @return encoded string of username:password
    */
    std::string Client::encode_credentials(void) {
        std::string usr_pwd = user_ + ":" + passwd_;
        std::string encoded;
        adnav::utils::Base64Encode(usr_pwd, &encoded);
        return encoded;
    }
} // namespace adnav::ntrip
