#include "command_history.h"

#include <iostream>
#include <memory>
#include <string>
#include <string_view>

namespace {

int failures = 0;

void expect(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

class SetValueCommand final : public ICommand {
public:
    SetValueCommand(
        std::string description,
        int& value,
        int before,
        int after,
        bool undo_succeeds = true,
        bool redo_succeeds = true
    )
        : m_description(std::move(description)),
          m_value(value),
          m_before(before),
          m_after(after),
          m_undo_succeeds(undo_succeeds),
          m_redo_succeeds(redo_succeeds) {}

    [[nodiscard]] std::string_view description() const override { return m_description; }

    bool undo() override {
        if (!m_undo_succeeds || m_value != m_after) {
            return false;
        }
        m_value = m_before;
        return true;
    }

    bool redo() override {
        if (!m_redo_succeeds || m_value != m_before) {
            return false;
        }
        m_value = m_after;
        return true;
    }

private:
    std::string m_description;
    int& m_value;
    int m_before;
    int m_after;
    bool m_undo_succeeds;
    bool m_redo_succeeds;
};

void apply(CommandHistory& history, int& value, int next, std::string description) {
    const int previous = value;
    value = next;
    history.record_applied(std::make_unique<SetValueCommand>(
        std::move(description),
        value,
        previous,
        next
    ));
}

void test_named_undo_and_redo() {
    CommandHistory history;
    int value = 0;
    expect(!history.is_dirty(), "new history is clean");
    expect(history.undo_description().empty(), "new history has no undo name");

    apply(history, value, 4, "Set Four");
    expect(history.can_undo(), "applied command can undo");
    expect(history.undo_description() == "Set Four", "undo exposes command name");
    expect(history.is_dirty(), "applied command makes history dirty");
    expect(history.undo(), "undo named command");
    expect(value == 0, "undo restores value");
    expect(history.redo_description() == "Set Four", "redo exposes command name");
    expect(!history.is_dirty(), "undo to initial saved state is clean");
    expect(history.redo(), "redo named command");
    expect(value == 4, "redo restores applied value");
}

void test_saved_position_and_divergence() {
    CommandHistory history;
    int value = 0;
    apply(history, value, 1, "First");
    apply(history, value, 2, "Saved Edit");
    history.mark_saved();
    expect(!history.is_dirty(), "mark saved records current state");

    expect(history.undo(), "undo from saved state");
    expect(history.is_dirty(), "undo away from saved state is dirty");
    expect(history.redo(), "redo to saved state");
    expect(!history.is_dirty(), "redo to saved state is clean");

    expect(history.undo(), "undo before divergent edit");
    apply(history, value, 3, "Divergent Edit");
    expect(!history.can_redo(), "divergent edit discards redo branch");
    expect(history.is_dirty(), "divergent state remains dirty");
    expect(history.undo(), "undo divergent edit");
    expect(value == 1, "undo divergent edit returns to branch point");
    expect(history.is_dirty(), "discarded saved state cannot be reached by undo");
}

void test_transaction_commit_and_cancel() {
    CommandHistory history;
    int value = 0;
    expect(history.begin_transaction("Move Selection"), "begin transaction");
    expect(!history.begin_transaction("Nested"), "nested transaction is rejected");
    apply(history, value, 2, "Move X");
    apply(history, value, 5, "Move Y");
    expect(history.is_dirty(), "applied transaction preview is dirty");
    expect(!history.can_undo(), "cannot undo while transaction is active");
    expect(history.commit_transaction(), "commit populated transaction");
    expect(history.undo_description() == "Move Selection", "transaction uses group name");
    expect(history.undo(), "undo transaction");
    expect(value == 0, "transaction undoes all commands in reverse order");
    expect(history.redo(), "redo transaction");
    expect(value == 5, "transaction redoes all commands in order");

    expect(history.begin_transaction("Canceled Move"), "begin canceled transaction");
    apply(history, value, 8, "Move X");
    expect(history.cancel_transaction(), "cancel transaction");
    expect(value == 5, "cancel rolls back applied commands");
    expect(history.undo_description() == "Move Selection", "cancel adds no history entry");

    expect(history.begin_transaction("Empty"), "begin empty transaction");
    expect(!history.commit_transaction(), "empty transaction creates no history entry");
    expect(!history.transaction_active(), "empty commit closes transaction");
}

void test_failed_commands_preserve_history_position() {
    CommandHistory history;
    int value = 0;
    value = 1;
    history.record_applied(std::make_unique<SetValueCommand>(
        "Cannot Undo",
        value,
        0,
        1,
        false,
        true
    ));
    expect(!history.undo(), "failed undo is reported");
    expect(value == 1, "failed undo leaves value unchanged");
    expect(history.can_undo(), "failed undo remains available");
    expect(history.undo_description() == "Cannot Undo", "failed undo keeps its name");

    history.clear();
    expect(!history.can_undo() && !history.can_redo(), "clear removes history entries");
    expect(!history.is_dirty(), "clear establishes a clean state");
}

} // namespace

int main() {
    test_named_undo_and_redo();
    test_saved_position_and_divergence();
    test_transaction_commit_and_cancel();
    test_failed_commands_preserve_history_position();

    if (failures != 0) {
        std::cerr << failures << " command history test(s) failed\n";
        return 1;
    }
    std::cout << "All command history tests passed\n";
    return 0;
}
