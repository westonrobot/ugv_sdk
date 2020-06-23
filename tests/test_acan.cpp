#include <iostream>
#include "wrp_sdk/asyncio/async_can.hpp"

using namespace westonrobot;

void parse_buffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    std::cout << "parser called" << std::endl;

    // mavlink::mavlink_status_t status;
    // mavlink::mavlink_message_t message;

    for (; bytes_received > 0; bytes_received--)
    {
        auto c = *buf++;

        // 	// based on mavlink_parse_char()
        // 	auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
        // 	if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
        // 		mavlink::_mav_parse_error(&m_status);
        // 		m_status.msg_received = mavlink::MAVLINK_FRAMING_INCOMPLETE;
        // 		m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_IDLE;
        // 		if (c == MAVLINK_STX) {
        // 			m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_GOT_STX;
        // 			m_buffer.len = 0;
        // 			mavlink::mavlink_start_checksum(&m_buffer);
        // 		}
        // 	}

        // 	if (msg_received != Framing::incomplete) {
        // 		log_recv(pfx, message, msg_received);

        // 		if (message_received_cb)
        // 			message_received_cb(&message, msg_received);
        // 	}
    }
}

int main(int argc, char *argv[])
{
    std::shared_ptr<ASyncCAN> canbus = std::make_shared<ASyncCAN>("can1");

    // canbus->set_receive_callback(parse_buffer);

    // if (canbus->is_open())
    //     std::cout << "can bus connected" << std::endl;

    struct can_frame frame;
    frame.can_id = 0x123;
    frame.can_dlc = 2;
    frame.data[0] = 0x11;
    frame.data[1] = 0x23;

    while (1)
    {
        // canbus->send_bytes(data, 3);
        canbus->send_frame(frame);
        sleep(1);
    }
}
