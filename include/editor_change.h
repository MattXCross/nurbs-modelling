#pragma once

struct EditorChange {
    bool selection{false};
    bool entities{false};
    bool geometry{false};
    bool properties{false};
    bool history{false};
    bool interaction_mode{false};
    bool hover{false};
    bool camera{false};
    bool viewport{false};

    void merge(EditorChange other) {
        selection = selection || other.selection;
        entities = entities || other.entities;
        geometry = geometry || other.geometry;
        properties = properties || other.properties;
        history = history || other.history;
        interaction_mode = interaction_mode || other.interaction_mode;
        hover = hover || other.hover;
        camera = camera || other.camera;
        viewport = viewport || other.viewport;
    }

    [[nodiscard]] bool empty() const {
        return !selection && !entities && !geometry && !properties && !history &&
            !interaction_mode && !hover && !camera && !viewport;
    }
};
