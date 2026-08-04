#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

class ICommand {
public:
    virtual ~ICommand() = default;
    [[nodiscard]] virtual std::string_view description() const = 0;
    [[nodiscard]] virtual bool undo() = 0;
    [[nodiscard]] virtual bool redo() = 0;
};

class CommandHistory {
public:
    void record_applied(std::unique_ptr<ICommand> command);
    [[nodiscard]] bool undo();
    [[nodiscard]] bool redo();
    [[nodiscard]] bool begin_transaction(std::string description);
    [[nodiscard]] bool commit_transaction();
    [[nodiscard]] bool cancel_transaction();
    void clear();
    void mark_saved();

    [[nodiscard]] bool can_undo() const;
    [[nodiscard]] bool can_redo() const;
    [[nodiscard]] bool transaction_active() const { return m_transaction.has_value(); }
    [[nodiscard]] bool is_dirty() const;
    [[nodiscard]] std::string_view undo_description() const;
    [[nodiscard]] std::string_view redo_description() const;

private:
    struct State {};
    struct Entry {
        std::unique_ptr<ICommand> command;
        std::shared_ptr<const State> before;
        std::shared_ptr<const State> after;
    };
    struct Transaction {
        std::string description;
        std::vector<std::unique_ptr<ICommand>> commands;
    };

    void append(std::unique_ptr<ICommand> command);

    std::vector<Entry> m_undo_stack;
    std::vector<Entry> m_redo_stack;
    std::optional<Transaction> m_transaction;
    std::shared_ptr<const State> m_current_state{std::make_shared<State>()};
    std::shared_ptr<const State> m_saved_state{m_current_state};
};
