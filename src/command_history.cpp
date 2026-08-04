#include "command_history.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

bool undo_commands(std::vector<std::unique_ptr<ICommand>>& commands) {
    for (auto command = commands.rbegin(); command != commands.rend(); ++command) {
        if (!(*command)->undo()) {
            for (auto restore = command; restore != commands.rbegin();) {
                --restore;
                (void)(*restore)->redo();
            }
            return false;
        }
    }
    return true;
}

bool redo_commands(std::vector<std::unique_ptr<ICommand>>& commands) {
    auto command = commands.begin();
    for (; command != commands.end(); ++command) {
        if (!(*command)->redo()) {
            while (command != commands.begin()) {
                --command;
                (void)(*command)->undo();
            }
            return false;
        }
    }
    return true;
}

class CompositeCommand final : public ICommand {
public:
    CompositeCommand(std::string description, std::vector<std::unique_ptr<ICommand>> commands)
        : m_description(std::move(description)), m_commands(std::move(commands)) {}

    [[nodiscard]] std::string_view description() const override { return m_description; }

    bool undo() override { return undo_commands(m_commands); }
    bool redo() override { return redo_commands(m_commands); }

private:
    std::string m_description;
    std::vector<std::unique_ptr<ICommand>> m_commands;
};

} // namespace

void CommandHistory::record_applied(std::unique_ptr<ICommand> command) {
    if (command == nullptr) {
        return;
    }
    if (m_transaction.has_value()) {
        m_transaction->commands.push_back(std::move(command));
        return;
    }
    append(std::move(command));
}

void CommandHistory::append(std::unique_ptr<ICommand> command) {
    auto next_state = std::make_shared<State>();
    m_undo_stack.push_back(Entry{std::move(command), m_current_state, next_state});
    m_current_state = std::move(next_state);
    m_redo_stack.clear();
}

bool CommandHistory::undo() {
    if (!can_undo()) {
        return false;
    }

    Entry entry = std::move(m_undo_stack.back());
    m_undo_stack.pop_back();
    if (!entry.command->undo()) {
        m_undo_stack.push_back(std::move(entry));
        return false;
    }
    m_current_state = entry.before;
    m_redo_stack.push_back(std::move(entry));
    return true;
}

bool CommandHistory::redo() {
    if (!can_redo()) {
        return false;
    }

    Entry entry = std::move(m_redo_stack.back());
    m_redo_stack.pop_back();
    if (!entry.command->redo()) {
        m_redo_stack.push_back(std::move(entry));
        return false;
    }
    m_current_state = entry.after;
    m_undo_stack.push_back(std::move(entry));
    return true;
}

bool CommandHistory::begin_transaction(std::string description) {
    if (m_transaction.has_value() || description.empty()) {
        return false;
    }
    m_transaction = Transaction{std::move(description), {}};
    return true;
}

bool CommandHistory::commit_transaction() {
    if (!m_transaction.has_value()) {
        return false;
    }
    Transaction transaction = std::move(*m_transaction);
    m_transaction.reset();
    if (transaction.commands.empty()) {
        return false;
    }
    append(std::make_unique<CompositeCommand>(
        std::move(transaction.description),
        std::move(transaction.commands)
    ));
    return true;
}

bool CommandHistory::cancel_transaction() {
    if (!m_transaction.has_value()) {
        return false;
    }
    if (!undo_commands(m_transaction->commands)) {
        return false;
    }
    m_transaction.reset();
    return true;
}

void CommandHistory::clear() {
    m_undo_stack.clear();
    m_redo_stack.clear();
    m_transaction.reset();
    m_current_state = std::make_shared<State>();
    m_saved_state = m_current_state;
}

void CommandHistory::mark_saved() {
    if (!m_transaction.has_value()) {
        m_saved_state = m_current_state;
    }
}

bool CommandHistory::can_undo() const {
    return !m_transaction.has_value() && !m_undo_stack.empty();
}

bool CommandHistory::can_redo() const {
    return !m_transaction.has_value() && !m_redo_stack.empty();
}

bool CommandHistory::is_dirty() const {
    return (m_transaction.has_value() && !m_transaction->commands.empty()) ||
        m_current_state != m_saved_state;
}

std::string_view CommandHistory::undo_description() const {
    return can_undo() ? m_undo_stack.back().command->description() : std::string_view{};
}

std::string_view CommandHistory::redo_description() const {
    return can_redo() ? m_redo_stack.back().command->description() : std::string_view{};
}
