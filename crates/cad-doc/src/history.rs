use crate::object::{CadObject, ObjectId};

#[derive(Debug, Clone)]
pub enum UndoAction {
    Added(ObjectId),
    Removed(ObjectId, CadObject),
    Modified {
        id: ObjectId,
        before: CadObject,
        after: CadObject,
    },
}

#[derive(Debug, Clone)]
pub struct UndoTransaction {
    pub label: String,
    pub actions: Vec<UndoAction>,
}

#[derive(Debug, Default)]
pub struct UndoHistory {
    undo_stack: Vec<UndoTransaction>,
    redo_stack: Vec<UndoTransaction>,
}

impl UndoHistory {
    pub fn push(&mut self, tx: UndoTransaction) {
        if tx.actions.is_empty() {
            return;
        }
        self.undo_stack.push(tx);
        self.redo_stack.clear();
    }

    pub fn pop_undo(&mut self) -> Option<UndoTransaction> {
        self.undo_stack.pop()
    }

    pub fn push_redo(&mut self, tx: UndoTransaction) {
        self.redo_stack.push(tx);
    }

    pub fn pop_redo(&mut self) -> Option<UndoTransaction> {
        self.redo_stack.pop()
    }

    pub fn push_undo_without_clearing_redo(&mut self, tx: UndoTransaction) {
        self.undo_stack.push(tx);
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }
}
