# Vibe Coding Refactoring - Implementation Roadmap

## 📋 Overview

This document provides a high-level roadmap for implementing the vibe coding architecture refactoring across all three packages.

## 🎯 Goals

1. **Reduce file sizes** to <300 LOC for better LLM comprehension
2. **Separate concerns** into core/adapters/nodes layers
3. **Improve testability** with 80%+ coverage for core logic
4. **Enable parallel development** through clear module boundaries
5. **Reduce AI hallucinations** by providing focused, understandable code

## 📊 Current vs Target State

### Current State (Problems)

| Package | Largest File | LOC | Main Issue |
|---------|--------------|-----|------------|
| voice | dialogue_node.py | 2313 | Everything in one file |
| perception | reflection_node.py | 877 | LLM + ROS + events mixed |
| mcp_tools | async_executor.py | 595 | Complex async logic |

**Total problematic LOC**: ~6000 lines across 6 files

### Target State (Solution)

| Package | Module Count | Avg LOC | Test Coverage |
|---------|--------------|---------|---------------|
| voice | 15 modules | ~200 | 80%+ |
| perception | 12 modules | ~180 | 80%+ |
| mcp_tools | 10 modules | ~190 | 80%+ |

**Total**: ~37 focused modules, each fully testable

## 🗓️ Implementation Timeline

### Phase 1: Foundation (Week 1-2)
**Goal**: Create architectural foundation and documentation

- [x] Week 1.1: Create VIBE_CODING_ARCHITECTURE.md
- [x] Week 1.2: Create refactoring plans for all packages
- [ ] Week 2.1: Set up test infrastructure
- [ ] Week 2.2: Create module templates and examples

**Deliverables**:
- ✅ Architecture documentation
- ✅ Detailed refactoring plans
- ⏳ Test framework setup
- ⏳ Module template examples

### Phase 2: Extract Core Logic (Week 3-6)
**Goal**: Extract business logic from large files (no breaking changes)

#### Week 3: rob_box_voice core modules
- [ ] Create `core/` structure
- [ ] Extract DialogueManager (~200 LOC)
- [ ] Extract CommandParser (~150 LOC)
- [ ] Extract SpeechFormatter (~150 LOC)
- [ ] Add unit tests for each

#### Week 4: rob_box_perception core modules
- [ ] Create `core/` structure
- [ ] Extract ContextBuilder (~200 LOC)
- [ ] Extract EventDetector (~200 LOC)
- [ ] Extract MemoryManager (~150 LOC)
- [ ] Add unit tests for each

#### Week 5: rob_box_mcp_tools core modules
- [ ] Create `core/` structure
- [ ] Extract TaskManager (~150 LOC)
- [ ] Extract Executor (~200 LOC)
- [ ] Move Registry to core/
- [ ] Add unit tests for each

#### Week 6: LLM layer extraction
- [ ] Voice: Extract streaming handler, tool call handler
- [ ] Perception: Extract reflection client, prompt manager
- [ ] MCP Tools: Extract chunk processor, tool accumulator
- [ ] Add unit tests with mocked APIs

**Deliverables**:
- ✅ All core modules extracted
- ✅ Unit tests passing (80%+ coverage)
- ✅ Original files still working (backward compatible)

### Phase 3: Refactor Nodes (Week 7-9)
**Goal**: Update nodes to use new modules

#### Week 7: Voice package
- [ ] Refactor dialogue_node.py (2313 → ~200 LOC)
- [ ] Refactor tts_node.py (858 → ~200 LOC)
- [ ] Refactor command_node.py (510 → ~150 LOC)
- [ ] Add integration tests

#### Week 8: Perception package
- [ ] Refactor reflection_node.py (877 → ~200 LOC)
- [ ] Refactor context_aggregator_node.py (719 → ~200 LOC)
- [ ] Update other nodes
- [ ] Add integration tests

#### Week 9: MCP Tools package
- [ ] Refactor adapters to use new structure
- [ ] Update llm_adapter.py
- [ ] Update server setup
- [ ] Add integration tests

**Deliverables**:
- ✅ All nodes refactored
- ✅ Integration tests passing
- ✅ Backward compatible ROS interfaces

### Phase 4: Documentation & Cleanup (Week 10-12)
**Goal**: Complete documentation and remove old code

#### Week 10: Documentation
- [ ] Update all package READMEs
- [ ] Add module READMEs
- [ ] Create architecture diagrams
- [ ] Write usage examples

#### Week 11: Testing improvements
- [ ] Increase coverage to 90%+
- [ ] Add edge case tests
- [ ] Performance testing
- [ ] Create test documentation

#### Week 12: Cleanup and final review
- [ ] Remove deprecated code paths
- [ ] Code review and refinements
- [ ] Update AGENT_GUIDE.md with new structure
- [ ] Create migration guide for developers

**Deliverables**:
- ✅ Complete documentation
- ✅ 90%+ test coverage
- ✅ Clean codebase
- ✅ Migration guide

## 🎯 Success Metrics

### Quantitative
- [ ] All files <300 LOC ✅ Target: 100%
- [ ] Test coverage ≥80% for core ✅ Target: 80%
- [ ] Test coverage ≥90% overall ✅ Target: 90%
- [ ] Module count: ~37 modules ✅ Target: 35-40
- [ ] Avg module size: ~190 LOC ✅ Target: <200

### Qualitative
- [ ] LLM can understand modules completely
- [ ] Easy to add new features
- [ ] Fast test suite (<2min for all tests)
- [ ] Clear module boundaries
- [ ] Good developer experience

## 🔄 Development Workflow

### For Each Module Extraction

1. **Plan** (30min)
   - Identify responsibility
   - Define interface
   - Plan tests

2. **Extract** (2-4hr)
   - Create new file
   - Copy/refactor logic
   - Add docstrings

3. **Test** (2-3hr)
   - Write unit tests
   - Aim for 80%+ coverage
   - Test edge cases

4. **Integrate** (1-2hr)
   - Update callers
   - Keep backward compatibility
   - Integration tests

5. **Document** (1hr)
   - Add module README
   - Update package docs
   - Add examples

**Total per module**: 6-10 hours

**For 37 modules**: ~300 hours (~2 months with 1 developer)

## 🤖 AI-Assisted Development Tips

### Effective Prompts

✅ **Good**: Specific, focused, with context
```
"Extract wake word detection logic from dialogue_node.py lines 150-200 
to core/wake_word_detector.py. Create WakeWordDetector class with:
- is_wake_word(text: str) -> bool method
- Configurable wake words list
Keep it under 100 LOC. Add tests to tests/unit/core/test_wake_word_detector.py"
```

✅ **Good**: Step-by-step breakdown
```
"Step 1: Create core/event_detector.py with EventDetector class
Step 2: Move state tracking logic from reflection_node.py lines 84-87
Step 3: Add should_react() method with cooldown logic
Step 4: Write unit tests without ROS dependencies
Each step should be a separate commit."
```

❌ **Bad**: Vague, too broad
```
"Make the code better"
"Refactor the perception package"
```

### Using Custom Agents

If you have access to task agents:

```python
# Use explore agent for codebase understanding
@task(agent_type="explore")
"Find all places where DialogueManager state is modified 
in dialogue_node.py and list them with line numbers"

# Use task agent for extraction
@task(agent_type="general-purpose")
"Extract streaming response handling from llm_adapter.py 
to streaming/chunk_processor.py following the interface 
defined in REFACTORING_PLAN_MCP_TOOLS.md"
```

## 📚 Key Documents Reference

| Document | Purpose | When to Use |
|----------|---------|-------------|
| [VIBE_CODING_ARCHITECTURE.md](./VIBE_CODING_ARCHITECTURE.md) | Core principles & patterns | Always - guiding document |
| [REFACTORING_PLAN_VOICE.md](./REFACTORING_PLAN_VOICE.md) | Voice package refactoring | Working on voice package |
| [REFACTORING_PLAN_PERCEPTION.md](./REFACTORING_PLAN_PERCEPTION.md) | Perception package refactoring | Working on perception |
| [REFACTORING_PLAN_MCP_TOOLS.md](./REFACTORING_PLAN_MCP_TOOLS.md) | MCP Tools refactoring | Working on mcp_tools |
| [AGENT_GUIDE.md](./AGENT_GUIDE.md) | AI prompting best practices | Writing prompts for AI |

## 🎓 Training & Onboarding

### For Human Developers

1. **Read** VIBE_CODING_ARCHITECTURE.md (30min)
2. **Review** one refactoring plan (30min)
3. **Practice** extract one small module (2hr)
4. **Review** with team (30min)

### For AI Agents

1. **Context**: Load VIBE_CODING_ARCHITECTURE.md
2. **Specific plan**: Load relevant REFACTORING_PLAN_*.md
3. **Focus**: Work on one module at a time
4. **Verify**: Run tests after each change

## 🚧 Risk Mitigation

### Risk: Breaking existing functionality
**Mitigation**:
- Keep old code working during transition
- Comprehensive integration tests
- Gradual migration of callers

### Risk: Tests become maintenance burden
**Mitigation**:
- Focus on core logic (high value)
- Use fixtures for common scenarios
- Keep tests simple and clear

### Risk: Over-engineering
**Mitigation**:
- Follow YAGNI principle
- Extract only what exists
- Don't add features during refactoring

### Risk: Lost momentum
**Mitigation**:
- Work on one package at a time
- Commit frequently (small wins)
- Track progress visually
- Celebrate milestones

## 📊 Progress Tracking

### Current Status

**Overall Progress**: 10% (Planning complete)

| Phase | Status | Progress |
|-------|--------|----------|
| Phase 1: Foundation | ✅ Complete | 100% |
| Phase 2: Core Extraction | ⏳ Not started | 0% |
| Phase 3: Node Refactoring | ⏳ Not started | 0% |
| Phase 4: Documentation | ⏳ Not started | 0% |

### Package Status

| Package | Core Modules | Tests | Nodes Refactored | Docs |
|---------|--------------|-------|------------------|------|
| voice | 0/15 | 0% | 0/6 | 0% |
| perception | 0/12 | 0% | 0/4 | 0% |
| mcp_tools | 0/10 | 0% | - | 0% |

**Next Milestone**: Extract first core module from voice package

## 🎉 Expected Benefits

### Short-term (Month 1-2)
- ✅ Better code organization
- ✅ Easier to understand for newcomers
- ✅ AI can help more effectively

### Medium-term (Month 3-6)
- ✅ Faster feature development
- ✅ Fewer bugs (better tests)
- ✅ Easy to onboard new developers

### Long-term (6+ months)
- ✅ Sustainable codebase
- ✅ High team velocity
- ✅ Low technical debt
- ✅ Clear upgrade paths

---

**Status**: Active  
**Last Updated**: 2025-01-20  
**Next Review**: When Phase 2 starts
