ARG  REPO=https://github.com/ssinad/dcvr-thesis

FROM registry.gitlab.com/ssinad/column-generation/cplex AS builder

COPY . /column-generation

WORKDIR /column-generation

RUN ln -s /ibm ./ibm \
    && make test_rooted_orienteering \
    && make test_cycle_orienteering \
    && make test_p2p_orienteering \
    && make test_dcvr
    # && make DEBUG_FLAG=-DNDEBUG test_dcvr 


FROM debian:buster-20210111-slim AS rooted-orienteering-stage

COPY --from=builder /column-generation/tests/test_rooted_orienteering.out ./orienteering

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./orienteering" ]

FROM debian:buster-20210111-slim AS cycle-orienteering-stage

COPY --from=builder /column-generation/tests/test_cycle_orienteering.out ./orienteering

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./orienteering" ]

FROM debian:buster-20210111-slim AS p2p-orienteering-stage

COPY --from=builder /column-generation/tests/test_p2p_orienteering.out ./orienteering

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./orienteering" ]


FROM debian:buster-20210111-slim AS dvrp-stage

COPY --from=builder /column-generation/tests/test_dcvr.out ./dcvr

LABEL org.opencontainers.image.source ${REPO}

ENTRYPOINT [ "./dcvr" ]


FROM builder AS testing-stage

RUN make run_tests