ARG  REPO=https://github.com/ssinad/dcvr-thesis

FROM registry.gitlab.com/ssinad/column-generation/cplex AS builder

COPY . /column-generation

WORKDIR /column-generation

RUN ln -s /ibm ./ibm \
    && make DEBUG_FLAG=-DNDEBUG test_rooted_orienteering \
    && make DEBUG_FLAG=-DNDEBUG test_cycle_orienteering \
    && make DEBUG_FLAG=-DNDEBUG test_dcvr 


FROM debian:buster-20210111-slim AS rooted-orienteering-stage

COPY --from=builder /column-generation/tests/test_rooted_orienteering.out ./orienteering

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./orienteering" ]

FROM debian:buster-20210111-slim AS cycle-orienteering-stage

COPY --from=builder /column-generation/tests/test_cycle_orienteering.out ./orienteering

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./orienteering" ]


FROM debian:buster-20210111-slim AS dvrp-stage

COPY --from=builder /column-generation/tests/test_dcvr.out ./dcvr

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./dcvr" ]


FROM builder AS testing-stage

RUN make clean && make run_tests