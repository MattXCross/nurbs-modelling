#pragma once

#include <memory>
#include <vector>

class ICommand {
public:
    virtual ~ICommand() = default;
    [[nodiscard]] virtual bool undo() = 0;
    [[nodiscard]] virtual bool redo() = 0;
};

class CommandHistory {
public:
    void record_applied(std::unique_ptr<ICommand> command);
    [[nodiscard]] bool undo();
    [[nodiscard]] bool redo();

    [[nodiscard]] bool can_undo() const { return !m_undo_stack.empty(); }
    [[nodiscard]] bool can_redo() const { return !m_redo_stack.empty(); }

private:
    std::vector<std::unique_ptr<ICommand>> m_undo_stack;
    std::vector<std::unique_ptr<ICommand>> m_redo_stack;
};
