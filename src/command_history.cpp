#include "command_history.h"

#include <utility>

void CommandHistory::record_applied(std::unique_ptr<ICommand> command) {
    if (command == nullptr) {
        return;
    }

    m_undo_stack.push_back(std::move(command));
    m_redo_stack.clear();
}

bool CommandHistory::undo() {
    if (m_undo_stack.empty()) {
        return false;
    }

    auto command = std::move(m_undo_stack.back());
    m_undo_stack.pop_back();
    command->undo();
    m_redo_stack.push_back(std::move(command));
    return true;
}

bool CommandHistory::redo() {
    if (m_redo_stack.empty()) {
        return false;
    }

    auto command = std::move(m_redo_stack.back());
    m_redo_stack.pop_back();
    command->redo();
    m_undo_stack.push_back(std::move(command));
    return true;
}
