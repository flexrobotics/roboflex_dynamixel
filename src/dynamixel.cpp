#include "roboflex_dynamixel/dynamixel.h"

namespace roboflex {
namespace dynamixelnodes {


// --- utility serialization functions ---

void write_state(flexbuffers::Builder& fbb, const DynamixelGroupState& state)
{
    fbb.Map("state", [&]() {
        for (DXLIdsToValues::value_type element: state.values) {
            DXLId dxl_id = element.first;
            string dxl_id_string = std::to_string(dxl_id);
            fbb.Map(dxl_id_string.c_str(), [&]() {
                DeviceValues control_table_values = element.second;
                for (DeviceValues::value_type control_table_entry: control_table_values) {
                    DXLControlTable control_table_key = control_table_entry.first;
                    int control_table_key_int = static_cast<std::underlying_type_t<DXLControlTable>>(control_table_key);
                    string control_table_key_string = std::to_string(control_table_key_int);
                    int control_table_value = control_table_entry.second;
                    fbb.Int(control_table_key_string.c_str(), control_table_value);
                }
            });
        }
    });
    
    // Write the timestamp pair.
    fbb.Double("t0", state.timestamp.t0);
    fbb.Double("t1", state.timestamp.t1);
}

void write_command(flexbuffers::Builder& fbb, const DynamixelGroupCommand& command)
{
    fbb.Map("command", [&]() {
        for (DXLIdsToValues::value_type element: command.values) {
            DXLId dxl_id = element.first;
            string dxl_id_string = std::to_string(dxl_id);
            fbb.Map(dxl_id_string.c_str(), [&]() {
                DeviceValues control_table_values = element.second;
                for (DeviceValues::value_type control_table_entry: control_table_values) {
                    DXLControlTable control_table_key = control_table_entry.first;
                    int control_table_key_int = static_cast<std::underlying_type_t<DXLControlTable>>(control_table_key);
                    string control_table_key_string = std::to_string(control_table_key_int);
                    int control_table_value = control_table_entry.second;
                    fbb.Int(control_table_key_string.c_str(), control_table_value);
                }
            });
        }
    });
}

DynamixelGroupState read_state(const flexbuffers::Map root_map)
{
    DXLIdsToValues values;
    auto control_map = root_map["state"].AsMap();
    auto control_map_keys = control_map.Keys();
    for (size_t i=0; i<control_map_keys.size(); i++) {
        string dxl_id_string = control_map_keys[i].AsString().str();
        DXLId dxl_id = std::stoi(dxl_id_string);
        auto dynamixel_control_map = control_map[dxl_id_string].AsMap();
        auto dynamixel_control_map_keys = dynamixel_control_map.Keys();
        for (size_t j=0; j<dynamixel_control_map_keys.size(); j++) {
            auto control_table_key = dynamixel_control_map_keys[j].AsString().str();
            int control_table_key_int = std::stol(control_table_key);
            DXLControlTable control_table_entry = DXLControlTable{control_table_key_int};
            int control_value = dynamixel_control_map[control_table_key].AsInt16();
            values[dxl_id][control_table_entry] = control_value;
        }
    }

    double t0 = root_map["t0"].AsDouble();
    double t1 = root_map["t1"].AsDouble();

    return DynamixelGroupState{values, TimestampPair{t0, t1}};
}

DynamixelGroupCommand read_command(const flexbuffers::Reference ref)
{
    DXLIdsToValues values;
    auto control_map = ref.AsMap();
    auto control_map_keys = control_map.Keys();
    for (size_t i=0; i<control_map_keys.size(); i++) {
        string dxl_id_string = control_map_keys[i].AsString().str();
        DXLId dxl_id = std::stoi(dxl_id_string);
        auto dynamixel_control_map = control_map[dxl_id_string].AsMap();
        auto dynamixel_control_map_keys = dynamixel_control_map.Keys();
        for (size_t j=0; j<dynamixel_control_map_keys.size(); j++) {
            auto control_table_key = dynamixel_control_map_keys[j].AsString().str();
            int control_table_key_int = std::stol(control_table_key);
            DXLControlTable control_table_entry = DXLControlTable{control_table_key_int};
            int control_value = dynamixel_control_map[control_table_key].AsInt16();
            values[dxl_id][control_table_entry] = control_value;
        }
    }

    return DynamixelGroupCommand{values, TimestampPair{0.0, 0.0}};
}


// --- DynamixelGroupStateMessage ---

DynamixelGroupStateMessage::DynamixelGroupStateMessage(const DynamixelGroupState& state):
    core::Message(ModuleName, MessageName)
{
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_state(fbb, state);
    });
    _state = get_state();
}

DynamixelGroupState DynamixelGroupStateMessage::get_state() const
{
    if (!_state_initialized) {
        _state = read_state(root_map());
        _state_initialized = true;
    }
    return _state;
}

void DynamixelGroupStateMessage::print_on(std::ostream& os) const
{
    DynamixelGroupState state = this->get_state();
    os << "<DynamixelGroupStateMessage ";
    state.print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}


// --- DynamixelGroupCommandMessage ---

DynamixelGroupCommandMessage::DynamixelGroupCommandMessage(const DynamixelGroupCommand& command):
    core::Message(ModuleName, MessageName)
{
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_command(fbb, command);
    });

    _command = get_command();
}

DynamixelGroupCommand DynamixelGroupCommandMessage::get_command() const
{
    if (!_command_initialized) {
        _command = read_command(root_val("command"));
        _command_initialized = true;
    }
    return _command;
}

void DynamixelGroupCommandMessage::print_on(std::ostream& os) const
{
    DynamixelGroupCommand command = this->get_command();
    os << "<DynamixelGroupCommandMessage ";
    command.print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}


// --- DynamixelStateCommandMessage ---

DynamixelStateCommandMessage::DynamixelStateCommandMessage(
    const DynamixelGroupState& state, 
    const DynamixelGroupCommand& command):
        core::Message(ModuleName, MessageName)
{
    flexbuffers::Builder fbb = get_builder();
    WriteMapRoot(fbb, [&]() {
        write_state(fbb, state);
        write_command(fbb, command);
    });

    _state = get_state();
    _command = get_command();
}

DynamixelGroupState DynamixelStateCommandMessage::get_state() const
{
    if (!_state_initialized) {
        _state = read_state(root_map());
        _state_initialized = true;
    }
    return _state;
}

DynamixelGroupCommand DynamixelStateCommandMessage::get_command() const
{
    if (!_command_initialized) {
        _command = read_command(root_val("command"));
        _command_initialized = true;
    }
    return _command;
}

void DynamixelStateCommandMessage::print_on(std::ostream& os) const
{
    DynamixelGroupState state = this->get_state();
    DynamixelGroupCommand command = this->get_command();
    os << "<DynamixelStateCommandMessage state:";
    state.print_on(os);
    os << " command:";
    command.print_on(os);
    os << " ";
    core::Message::print_on(os);
    os << ">";
}


// --- DynamixelGroupControllerNode ---

void DynamixelGroupControllerNode::receive(core::MessagePtr m)
{
    const std::lock_guard<std::recursive_mutex> lock(last_message_mutex);
    last_message = m;
}

void DynamixelGroupControllerNode::child_thread_fn()
{
    auto f = [this](const DynamixelGroupState& state, DynamixelGroupCommand& command) {

        bool should_continue = !this->stop_requested();

        if (should_continue) {
            const std::lock_guard<std::recursive_mutex> lock(last_message_mutex);
            command.values = this->readwrite_loop_function(state, last_message);
            this->signal(std::make_shared<DynamixelStateCommandMessage>(state, command));
        }

        return should_continue;
    };

    this->controller->run_readwrite_loop(f);
    this->controller->freeze();
}


// --- DynamixelGroupNode ---

DynamixelGroupNode::DynamixelGroupNode(
    DynamixelGroupController::Ptr controller,
    const string& name):
        core::RunnableNode(name),
        controller(controller)
{

}

void DynamixelGroupNode::receive(core::MessagePtr m)
{
    const std::lock_guard<std::recursive_mutex> lock(last_command_message_mutex);
    last_command_message = std::make_shared<DynamixelGroupCommandMessage>(*m);
}

bool DynamixelGroupNode::readwrite_loop_function(
    const DynamixelGroupState& state, 
    DynamixelGroupCommand& command)
{
    bool should_continue = !this->stop_requested();

    if (should_continue) {
        {
            // Get the last command message
            const std::lock_guard<std::recursive_mutex> lock(last_command_message_mutex);

            // Populate the command from the last command message
            if (last_command_message == nullptr) {
                command.should_write = false;
            } else {
                command.should_write = true;
                command.values = last_command_message->get_command().values;
            }
        }

        // Signal the state
        this->signal(std::make_shared<DynamixelGroupStateMessage>(state));
    }

    return should_continue;
}

void DynamixelGroupNode::child_thread_fn()
{
    auto f = [this](const DynamixelGroupState& state, DynamixelGroupCommand& command) {
        return this->readwrite_loop_function(state, command);
    };

    this->controller->run_readwrite_loop(f);
    this->controller->freeze();
}


// --- DynamixelRemoteController ---

DynamixelRemoteController::DynamixelRemoteController(
    const string& name):
        core::Node(name)
{

}

void DynamixelRemoteController::receive(core::MessagePtr m)
{
    if (m->message_name() == DynamixelGroupStateMessage::MessageName) {
        auto state = DynamixelGroupStateMessage(*m).get_state();
        DXLIdsToValues commanded_values = this->readwrite_loop_function(state);
        DynamixelGroupCommand command = DynamixelGroupCommand{commanded_values};
        this->signal(std::make_shared<DynamixelGroupCommandMessage>(command));
    }
}


// --- DynamixelRemoteFrequencyController ---

DynamixelRemoteFrequencyController::DynamixelRemoteFrequencyController(
    const float frequency_hz,
    const string& name):
        nodes::FrequencyGenerator(frequency_hz, name)
{

}

void DynamixelRemoteFrequencyController::receive(core::MessagePtr m)
{
    if (m->message_name() == DynamixelGroupStateMessage::MessageName) {
        this->state = DynamixelGroupStateMessage(*m).get_state();
    }
}

void DynamixelRemoteFrequencyController::on_trigger(double wall_clock_time)
{
    if (this->state.timestamp.t0 != 0) {
        DXLIdsToValues commanded_values = this->readwrite_loop_function(this->state);
        DynamixelGroupCommand command = DynamixelGroupCommand{commanded_values};
        this->signal(std::make_shared<DynamixelGroupCommandMessage>(command));
    }
}

} // dynamixelnodes
} // roboflex
